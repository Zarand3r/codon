# IPC & Fault-Tolerance Design

This document describes how interprocess communication (IPC) is set up in this
flight-software codebase (the `bullwinkle` framework and the `flight` runtimes
built on top of it), and how fault tolerance / triple-modular redundancy (TMR)
is layered on top of that IPC.

> Scope note: this is reconstructed from the code present in the tree. The core
> data model (`Slate*`), the transport layer (`io/*`), the telemetry relay, and
> the runtime wiring (`FtRuntime`, `DroneFtRuntime`, `BasicControl`) are all
> here. Several implementation headers that the runtime *uses* are referenced
> but not included in this slice of the repo — notably `SlateSharer`,
> `SlateSyncer`, `FtSync`, `FtSimpleDataSharer`, `NodeIoManager`,
> `FtChannelManager`. Where this doc describes their behavior, it is inferred
> from how `FtRuntime`/`BasicControl` construct and drive them. Those spots are
> flagged.

---

## 1. The big picture

The system is a **fault-tolerant, triple-string flight controller**. A single
logical "flight computer" is implemented as **three redundant processes called
*strings* (`a`, `b`, `c`)**, which may run on three separate physical computers
(or, for some node types, be collocated on one). The three strings run *the same
deterministic control code* on *the same voted inputs* so they produce *the same
outputs*, and any one of them can fail without taking down the vehicle.

Everything hangs off one central data abstraction: the **Slate**.

```
   physical sensors / hardware (FPGA, ADC)
              │  FirmwareComm (DMA)
              ▼
   ┌─────────────────────┐     UDP (share/reshare)     ┌──────────────┐
   │   STRING a (proc)   │◄───────────────────────────►│  STRING b/c  │
   │                     │                              │  (procs)     │
   │  Slate (local mem)  │   input data sharing + vote  └──────────────┘
   │   ├ static          │
   │   ├ sync   ◄─────────── SlateSyncer hotsync (UDP) ──────────►
   │   ├ nonsync         │
   │   └ cyclic          │   SlateSharer (UDP) → median vote → control slate
   └─────────┬───────────┘
             │ TelemetryRelay (UDP/TCP, BWP frames)
             ▼
        ground / RF
```

---

## 2. The Slate: the data model behind IPC

The **Slate** (`vehicle/src/bullwinkle/all/Slate.h`,
`SlateBuilder.h`, `SlateMemory.h`, `SlateLayout.h`) is *"a centralized data
storage engine. Subsystems can store their state structures in the Slate, and
retrieve them by ID."* It is the unit of everything that IPC moves around.

Key properties that make it the IPC substrate:

- **All state lives in the Slate.** Rule #1 in `Slate.h`: *"DO NOT keep private
  state in your objects outside of Slate."* Because everything is in the Slate,
  the same machinery can telemeter it, command it, save/restore it, and
  **synchronize it between computers**.
- **No pointers allowed** (Rule #4). Slate data must be position-independent so
  it is *"valid between runs and across different computers, which makes
  synchronizing and exchanging data in a generic manner [possible]"*.
- **Two phases**: a *build phase* (`SlateBuilder`) allocates a fixed memory
  layout; a *run phase* (`Slate`) accesses elements by token/ID. Once built, the
  layout is frozen, which lets the layout be hashed and compared across peers
  (`get_shard_layout_hash`, `compute_hash`, `compute_shard_deltas`).
- **Elements are addressed by hierarchical string paths** (e.g.
  `heap.bytes`, `rtc.nav_time_to_persist`, `node_io.*`). These paths are the
  closest thing the system has to pub/sub "topics" (see §6).

### 2.1 Is the Slate shared memory?

**No — not OS shared memory.** Slate storage is ordinary **process-local heap**:
`SlateMemory` holds one `AlignedBuffer shard_table[num_slate_shard_t]`
(`SlateMemory.h`), allocated from the normal heap. There is no `shm_open`,
no `mmap(MAP_SHARED)`, no cross-process shared pages for Slate data.

The only `mmap` in the tree is in `DroneFtRuntime.cc`, and it maps **FPGA
register banks** (`/sys/devices/platform/amba_pl/.../mmap`) — that is
memory-mapped *hardware I/O*, not interprocess shared memory.

So a Slate is private to its process. Cross-process and cross-computer data
movement is done by **serializing Slate memory and sending it as datagrams**.

---

## 3. Shards: per-element memory policy

Each Slate element is created in a **shard**, which defines its "memory policy."
From `FtRuntime::init_post_slate_build_early()` there are 7 shards
(`num_slate_shard_t == 7`):

| Shard | Telemetered? | Meaning |
|-------|-------------|---------|
| `shard_static` | yes | Constant / configuration data. |
| `shard_sync` | yes | **Synchronized control state** — kept bit-identical across all three strings (this is what `SlateSyncer` hashes & hotsyncs). |
| `shard_sync_no_telem` | no | Same as `sync`, but not sent to telemetry. |
| `shard_nonsync` | yes | **Per-process local state** — *not* synchronized between strings (e.g. heap stats, local timers, non-synced commanding). |
| `shard_nonsync_no_telem` | no | Local, non-telemetered. |
| `shard_cyclic` | yes | **Reset every control frame** — blown away by `slate.roll_frame()` at end of cycle. |
| `shard_cyclic_no_telem` | no | Cyclic, non-telemetered. |

Permissions enforce the redundancy boundary: only the *control* slate may create
elements in the **sync** shard; runtime/local slates are denied
`slate_permission_c_sync` (`FtRuntime::init_pre_control_early`). Conversely the
control slate is restricted from creating in nonsync shards. This is what keeps
"shared, voted" state separate from "private, per-string" state.

`roll_frame()` (`Slate.h`) replaces the cyclic shard from a template each frame;
that is why all cyclic-shard work must finish before the end of `dispatch`.

---

## 4. The transport layer (this is where the network lives)

All actual IPC bytes travel over **sockets**, wrapped by the `io/` channel
abstraction:

- **`DgramChannel` / `FdDgramChannel`** (`io/FdDgramChannel.h`) — a buffered,
  duplex datagram channel backed by a file descriptor. Emits a `read_sig` when
  data arrives and a `write_sig` when buffer space frees up (this is the
  in-process plumbing — see §6). Supports UDP sockets, connected/unconnected,
  device files, and `tun` devices.
- **`UdpConnection`** (`io/UdpConnection.h`) — a `DgramConnection` that sends
  datagrams to a UDP port, auto-reconnecting. Supports IPv4/IPv6, multicast,
  broadcast. **This is the primary cross-process / cross-computer transport.**
- **`TcpConnection` / `TcpServer` / `TcpP2PServerConnection` /
  `TcpMultiServerConnection`** (`io/`) — stream transport, used mainly by
  telemetry destinations and point-to-point server connections.
- **`Server`** (`Server.h`) — generic connection-oriented server framework
  (accept/prune connections).

### 4.1 Service directory / routing (naming)

Endpoints are not hardcoded. Code resolves a **service name → host/port/proto**
through a `service_directory()` lookup that returns a `Service` with a `proto`
field (`udp_proto`, `tcp_proto`). Examples in `DroneFtRuntime.cc`:

```cpp
service_directory().lookup(timestamp_service, service);
FswAbortIfNot(service.proto == udp_proto, false);
...
service_directory().lookup(Satellite::alert_buffer_output_service, alerts_service);
FswAbortIf(alerts_service.proto != udp_proto, false);
output_connection->open(alerts_service.host_name, alerts_service.port);
```

A separate `node_directory()` maps the redundant nodes to network addresses;
`verify_node_directory_segments(...)` validates that every node's IP lands in a
known vehicle network segment (ground / rf / satellite / utility / payload).
This "service directory and routing" layer is item #3 in `TODO.md`.

### 4.2 Are shared memory and UDP "two separate paths that switch"?

**No.** There is exactly **one** cross-process path: **message passing over
datagram sockets (UDP).** There is no shared-memory IPC path, so there is
nothing to switch between.

The thing that *looks* like a second path is the distinction between:

- **Intra-process** communication — direct `Slate` memory reads/writes plus the
  `Signal` observer mechanism (§6). No sockets; everything is in one address
  space.
- **Inter-process / inter-computer** communication — always serialized over
  datagram channels.

Even **collocated** strings (`triple_string_colloc_ft_node`) and even a string
sharing data *with itself* use the datagram machinery rather than shared memory.
See `FtRuntime::handle_local_share_read()`: the node's own local-share message is
read off a `DataDgramChannel` and re-injected into the data-sharing pipeline as
if it had arrived from the network:

```cpp
// "make it look like the data came from our node via the node IO manager"
channel->read_sig.connect(slot_bind(
    make_slot(*this, &FtRuntime::handle_local_share_read), node_id));
```

So the mental model is: **one process == one Slate in local memory; crossing a
process boundary always means a datagram.** The choice of UDP vs TCP is per
*service* (from the service directory), not a runtime fallback between shm and
net.

---

## 5. The three cross-process data movers

There are three distinct subsystems that move Slate data between strings, each
serving a different purpose. All three ride on the datagram transport.

### 5.1 Input data sharing — `SlateSharer` + `FtSimpleDataSharer` + `NodeIoManager`

Purpose: get *the same input data* into all three strings so their identical
control code produces identical results ("cross-strapping").

Wiring (`FtRuntime::create_input_sharing_system`, `init_pre_slate_build_late`):

- `NodeIoManager` opens the network connections that bring in external/sensor
  data and exposes a `data_sig`.
- Each string serializes its own locally-acquired inputs with a
  **`SlateSharerSender`** (configured by a `sharer_config_v` — a *list* of slate
  element paths to share) and emits them.
- **`FtSimpleDataSharer`** exchanges those input datagrams between strings over
  dedicated **UDP** connections in a ring (`left_share_conn`,
  `right_share_conn`, plus `left_reshare_conn`/`right_reshare_conn` for a second
  "reshare" pass). Messages are signed via the `Keychain`. The result is fed
  into the `FtChannelManager` (`read_input`).
- On the control side (`BasicControl`), each peer's data lands in a per-string
  slate (`TripleString<SlateBuilder> slate_shared` → `a`/`b`/`c`) via three
  **`SlateSharerReceiver`s** (`slate_receiver_a/b/c`).

> `SlateSharer`/`SlateSharerSender`/`SlateSharerReceiver` headers are referenced
> but not in this slice; behavior above is inferred from the constructors/calls.

### 5.2 Input voting — `SlateCombiner`

Purpose: turn three (possibly disagreeing) copies of each input into one trusted
value.

`SlateCombiner` (`SlateCombiner.h`) reads from up to **3 source slates** and
writes a single destination (the control slate). Per element it computes a
**median**: *"a 3/3, 2/2, 1/1 median of the source Slate values on an
element-by-element basis depending on freshness, or a direct copy of all the
values from the first fresh source."*

- Each source has a **freshness** element; a source that hasn't updated within
  `stale_threshold` cycles is considered disconnected and dropped from the vote.
- A **`downselect`** mode exists to avoid "tearing" across elements that need
  inter-element coherency: if there's no clear majority it takes the first
  connected source wholesale instead of per-element medians.

In `BasicControl`, `slate_combiner_control` combines the three `slate_shared`
inputs into `slate_control`. This is the literal TMR input vote.

### 5.3 State sync / hotsync — `SlateSyncer`

Purpose: let a string that **rebooted or desynced** recover the entire
synchronized control state (the **sync shard**) from a healthy peer, and detect
when strings have drifted apart.

Wiring (`FtRuntime`):

- `SlateSyncer` is created with a `slate_syncer_duplex_mode` and its **own
  dedicated `FdBag`** so its (potentially large) transfers can be dispatched
  separately from normal I/O.
- `init_inputs()` is given the three per-string shared slates plus a median
  slate; `init_outputs()` exposes our own state.
- Each cycle: `dispatch_check()` detects desync; `dispatch_share()` hashes the
  sync shard and prepares it for peers; `dispatch_process()` applies an incoming
  hotsync. It relies on `Slate`'s shard hashing primitives
  (`get_shard_layout_hash`, `compute_hash`, `compute_shard_deltas`,
  `swap_shard_buffer`).
- If `slate_syncer_synchronous_send` is set, the sync shard is flushed via a
  zero-timeout `select_absolute(..., fd_write_ev, ...)` ("select once") right
  after it's prepared, so the big transfer overlaps with other output emission.

Note `DroneFtRuntime::init_runtime_post_slate_build()` explicitly
`enable_transfer()` + `enable_hotsync()`; the value of
`single_peer_hotsync_enabled` and `force_disable_transfer` are themselves slate
elements read each cycle, i.e. **hotsync policy is commandable**.

---

## 6. Is communication topic-based pub/sub?

Two different mechanisms, neither is a generic broker-style topic bus:

1. **In-process: `Signal`/`Slot` (observer pattern).** `Signal.h` is a
   signal/slot library: a `Signal<...>` is *"a set of functions that can be
   called"*; consumers `connect()` a `Slot`, producers `emit()`. This is true
   publish/subscribe **within one process** (e.g. `channel->read_sig`,
   `data_sharer->shared_data_sig`, `bootstrapper->full_sync_established_sig`),
   but it is function-call delivery, not named topics.

2. **Cross-process: configured element lists, not subscriptions.** A
   `SlateSharer` is handed an explicit `sharer_config_v` — a static list of
   slate element **paths** — at init time. There is no runtime "subscribe to
   topic X." Telemetry is organized into **flows** and **groups**
   (`telem_group_t`, `telem_id_t` in `TelemetryRelay.h`); commands are addressed
   by element **name or hash** (`SlateCommandInterface`, `set_*_by_hash`).

So the **element path is the nearest analog to a topic**, but distribution is
configuration-driven (sender knows its element list, receiver knows where to put
them), not a dynamic pub/sub registry.

---

## 7. Does each topic get its own message, or does one message carry many?

**One datagram carries many elements/"topics." It is never one-message-per-topic.**

- **Input sharing**: a `SlateSharerSender` packs its whole configured
  `sharer_config_v` element list (a slice of a shard) into a datagram. The
  buffer size comes from `node_configs.get_node_output_buffer_size(...)`, i.e.
  it is sized to hold the full set, not one value.
- **State sync**: `SlateSyncer` transfers an **entire shard** (the sync shard)
  as one logical (large) transfer — explicitly noted as *"usually very large"*
  in `FtRuntime`.
- **Telemetry**: `TelemetryRelay` frames many channels into **BWP** datagrams
  (`BwpFramer`, `BwpWriter`, `bwp_mtu`), batching by flow.
- **Commands**: a single `multi_command_t` batch can set up to 50 elements at
  once (`SlateCommandInterface::multi_command_v`).

Granularity is therefore **shard / element-set per message**, chosen for cache
coherency and network efficiency, not per-element.

---

## 8. Fault tolerance / TMR — how the three strings stay redundant

Yes — there is genuine **triple-modular redundancy** in which **inputs, state,
and outputs are all replicated and reconciled across three processes.** It is
built from four cooperating synchronization layers.

### 8.1 Node types (`FtRuntime::ft_node_type_t`)

| Node type | input sync | output sync | slate syncer | firmware comm |
|-----------|:---------:|:-----------:|:------------:|:-------------:|
| `triple_string_ft_node` | ✓ | – | ✓ | ✓ |
| `triple_string_colloc_ft_node` | ✓ | – | ✓ | ✓ (collocated on a shared computer) |
| `triple_string_guest_ft_node` | ✓ | – | ✓ | – (network-only "VM") |
| `triple_dual_master_ft_node` | ✓ | ✓ | – | – |
| `triple_dual_slave_ft_node` | – | ✓ | – | – |

(Derived in `FtNodeProperties`.) A "triple-dual" pair adds **output
synchronization** on top of the triple-string scheme. Drone FCs run as
`triple_string_ft_node` (or `_guest` when they have no FirmwareComm).

### 8.2 The four synchronization layers

1. **Time sync — `FtSync`.** Phase-locks the control cycle across all strings (a
   distributed PLL). `FtSync::dispatch()` runs *first* every cycle and sets the
   EventLoop's control time, so all strings step in lockstep. `satfc1` is the
   designated time-sync initiator (`DroneFtRuntime::create_bootstrapper`).
   `dispatch()` returns `nano_t_min` precisely so *"FtSync [can] govern control
   cycle timing."*

2. **Input sync (§5.1 + §5.2)** — share each string's inputs, then median-vote
   them so all strings compute on **identical inputs**. Inputs are
   cryptographically signed (`Keychain`).

3. **Identical synced execution.** In `FtRuntime::dispatch()`, once
   `bootstrapper->is_synced()`, every string runs `control->dispatch_synced()`.
   Because inputs are voted-identical and the code is deterministic, **all
   strings produce identical sync-shard state and identical outputs.** Telemetry
   from "muxed" groups must even produce identical BWP framing across strings
   (`get_muxed_telemetry_groups`).

4. **Output sync (triple-dual).** When `use_output_sync` is set, the data
   sharer's output is *not* wired straight to the channel manager; it is instead
   routed to an output comparator/forwarder for **output voting** between the
   master and slave halves (`create_input_sharing_system`:
   `if (!props.use_output_sync) data_sharer->shared_data_sig.connect(channel_manager...)`).

5. **State recovery — `SlateSyncer` hotsync (§5.3).** A rebooted/desynced string
   pulls the whole sync shard from a peer and rejoins, governed by the
   `FtBootstrapper` state machine.

### 8.3 The control cycle (one tick of TMR)

From `FtRuntime::dispatch()` → `dispatch_nonsynced_early()` /
`dispatch_synced()` / `dispatch_nonsynced_late()`:

```
EARLY (per string, not yet voted):
  1. FtSync.dispatch          → agree on control time (time sync)
  2. SlateSyncer.dispatch_process → apply any hotsync (state recovery)
  3. FirmwareComm read/write  → talk to FPGA/ADC hardware (DMA)
  4. slate_sender_local.share → broadcast my inputs to peers   ── UDP ──►
  5. data_sharer.dispatch     → exchange + vote peers' inputs  ◄── UDP ──
  6. SlateSyncer.dispatch_check → desync detection
  7. bootstrapper.dispatch    → advance sync state machine
  8. FtSync.send_syncs        → emit time-sync beacons         ── UDP ──►

SYNCED (only if is_synced — identical on all strings):
  9. control->dispatch_synced → state machine, alarms, control law, outputs

LATE (per string):
 10. TelemetryRelay.dispatch  → frame + emit telemetry         ── UDP/TCP ──►
 11. SlateSyncer.dispatch_share → hash sync shard, offer hotsync to peers
     (+ optional "select once" to start the big sync-shard send)
 12. slate.roll_frame         → wipe cyclic shard for next cycle
 13. channel_manager.flush_inputs
```

Non-synced nodes that skip step 5 instead `fswsleep(ds_parallel_sleep_time)` to
stay phase-aligned with peers that *do* share — preserving lockstep timing even
across asymmetric node types.

---

## 9. Command and telemetry paths (the other IPC users)

- **Commands in** arrive as datagrams. `ExternalCommandDispatcher` deframes →
  time-filters → command-filters → arms → dispatches to handlers. The
  `ExternalCommandSlateHandler` drives a **`SlateCommandInterface`**, which sets
  slate elements **by name or hash** (`SlateCommandInterface.h`). There are two
  command platforms: a **synced** one in `BasicControl`
  (`ground_cmd_dispatcher_synced`, writes the sync shard) and a **non-synced**
  one in `FtRuntime` (`gnd_cmd_dispatcher_nonsynced`, writes `shard_nonsync`).
- **Telemetry out** is the reverse: `TelemetryRelay` claims flows, frames slate
  data into BWP datagrams, rate-limits, and writes to UDP/TCP destinations
  resolved through the service directory. Some flows are **redirected to local
  writers** instead of the network (e.g. the alert buffer in
  `DroneFtRuntime::create_local_telemetry_connections`).

---

## 10. Summary answers

- **Is IPC based on shared memory?** No. The Slate lives in process-local heap;
  there is no `shm`/`MAP_SHARED` IPC. (The only `mmap` is FPGA hardware
  registers.)
- **Does it use UDP to synchronize between machines?** Yes. Cross-string /
  cross-computer sync — input sharing, time sync, and state/hotsync — all use
  **UDP datagrams**; telemetry/commands use UDP and TCP per the service
  directory.
- **Are shared memory and UDP two separate paths that switch?** No. There's one
  cross-process path (datagrams). Shared memory isn't used for IPC, so there's
  no switch. The only "two paths" are *in-process* (direct Slate + `Signal`) vs
  *cross-process* (datagrams); even collocated/self traffic goes through
  datagrams.
- **Is it topic-based pub/sub?** Partially. `Signal`/`Slot` is in-process
  observer pub/sub; cross-process distribution is **configuration-driven element
  lists**, with the hierarchical **element path** acting as the topic-like name.
  There is no dynamic topic-subscription broker.
- **One message per topic, or many topics per message?** **Many per message** —
  sharers pack a configured element list, the syncer ships a whole shard,
  telemetry batches flows into BWP frames, and commands batch up to 50 elements.
- **Is there true TMR across 3 processes?** Yes. Three strings (`a`/`b`/`c`)
  run identical deterministic control on **median-voted inputs**, keep their
  **sync shard** bit-identical (with hotsync recovery), and — in triple-dual
  configurations — additionally **vote outputs**. Time sync keeps all three in
  lockstep.
```
