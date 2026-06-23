# Distributed Flight Control — System Design with Mechanisms

> This document describes the design of a fault-tolerant, triple-redundant
> real-time flight controller (the `bullwinkle` framework and the `flight`
> runtimes built on top of it). It begins from a conceptual model and then
> grounds every mechanism in the concrete code that implements it — exact type
> names, the comments the authors wrote, the constants, and the file locations —
> so the document can be both read top-to-bottom and used as a map into the
> source.
>
> Source roots referenced throughout:
> - Core data model & transport: `vehicle/src/bullwinkle/all/`
> - Fault-tolerant runtime & control: `vehicle/src/flight/common/all/`
> - Satellite flight computer specialization: `vehicle/src/flight/sat/all/flight-computer/`
> - Ground-link command/telemetry plumbing: top-level (`TelemetryRelay`,
>   `SlateCommandInterface`, `CommandQueueClient`, `gRPC_*`, …)
>
> The redundancy framework (`bullwinkle` + `flight/common`) is **vehicle-agnostic**
> — it makes no assumption about what is being controlled. The satellite-specific
> symbols quoted below (`flight/sat`, `Satellite::…`, `satfc`/`satgps`, the FPGA
> register map) are the *current vehicle layer*; swapping them for a drone layer
> leaves the redundancy model intact (see `ROADMAP.md`, Track B).

---

## Purpose and redundancy model

The system is a **fault-tolerant real-time vehicle controller**. The control
logic runs **three times in parallel** on three independent units — called
**"strings"** (`a`/`b`/`c`) — ideally on separate physical computers. All three
run the same deterministic logic, on the same data, in lockstep, so they
independently reach the same decisions. Any one unit can fail or reboot without
interrupting control, and a recovered unit can rejoin. The redundancy is
end-to-end: **inputs, internal state, and outputs are each replicated and
cross-checked**, not just the final result.

Each leg of that end-to-end cross-check has a dedicated mechanism below:

- **Inputs** — replicated by ring-sharing and cross-checked by **median voting**
  ("Input agreement and voting").
- **Internal state** — the replicated (`sync`) state is **continuously hashed and
  byte-compared** across peers and **recovered wholesale** on divergence ("State
  recovery").
- **Outputs** — in the strongest (triple-dual) configuration, routed through a
  **comparator and voted** before they take effect ("Identical computation,
  optional output voting").

The four cooperating mechanisms under "Keeping the three units in agreement" are
how three independent processes are made to behave as one.

The three strings are represented in code by a `TripleString<T>` template with
`.a`/`.b`/`.c` members — e.g. `TripleString<SlateBuilder> slate_shared` in
`BasicControl.h:194`. A unit's identity comes from `ident.role_inst` plus a
single string character `ident.string[0]`; the per-string shared-slate name is
literally `ident.role_inst + string` (`FtRuntime::shared_slate_name`,
`FtRuntime.cc:1675`), producing names like `satfc1a`, `satfc1b`, `satfc1c`.

```
   physical sensors / hardware (FPGA, ADC)
              │  FirmwareComm (DMA / mmap'd registers)
              ▼
   ┌─────────────────────┐   UDP share / reshare ring   ┌──────────────┐
   │   STRING a (proc)   │◄───────────────────────────►│  STRING b/c  │
   │                     │   (signed input datagrams)   │   (procs)    │
   │  Slate (local heap) │                              └──────────────┘
   │   ├ static          │   SlateSharer → median vote → control slate
   │   ├ sync   ◄──────────── SlateSyncer hotsync (UDP, whole shard) ──────►
   │   ├ nonsync         │
   │   └ cyclic          │
   └─────────┬───────────┘
             │ TelemetryRelay (UDP/TCP, BWP frames, rate-limited)
             ▼
        ground / RF  ◄── gRPC command pump / telemetry ──►
```

---

## A single named model of all state — the **Slate**

Every value in the system — sensor readings, configuration, control variables,
command targets, health counters — lives in **one structured data model**, the
**Slate** (`vehicle/src/bullwinkle/all/Slate.h`). The Slate's own header
describes it as:

> *"A centralized data storage engine. Subsystems can store their state
> structures in the Slate, and retrieve them by ID. Data stored in the slate can
> be read by telemetry, set by command, synchronized between computers, saved to
> file, etc."* (`Slate.h`)

Treating all state uniformly is exactly what lets the same machinery record it,
telemeter it, accept commands against it, snapshot it, and synchronize it
between units. The Slate's design rules (quoted from `Slate.h`) make this
explicit:

1. **"DO NOT keep private state in your objects outside of Slate. This will make
   it impossible to telemeter / save / restore / synchronize your program."**
2. **"PLEASE DO store your static/configuration data in Slate (and use it from
   there!). This allows it to be updated/telemetered/etc. and may prevent having
   to reboot vehicle computers unnecessarily."**
3. *"DO NOT store the reference you get from the Slate `load()` calls. Once they
   go out of scope where you load them, they are invalidated."*
4. **"DO NOT store pointers in the Slate (that includes ST containers!). They are
   invalid between runs and across different computers, which makes synchronizing
   and exchanging data in a generic manner impossible."**

Elements are addressed by **hierarchical string paths** (e.g. `heap.bytes`,
`rtc.nav_time_to_persist`, `node_io.*`) during the build phase. These paths are
the closest thing the system has to pub/sub "topics" (see
[Communication style](#communication-style-and-message-granularity)). At run
time those paths have been resolved to `SlateToken<T>` handles wrapping a single
`slate_element_t` ID — you `bind()` a path to a token once and thereafter access
by token.

### Build phase, then frozen

The model's layout is **fixed once at startup and then frozen**. The Slate is
split into two interfaces (`Slate.h`):

> *"The Slate is split into two halves: Slate and SlateBuilder. The Slate is the
> run-time interface, while the SlateBuilder is the initialization time
> interface."*

- **Build phase** (`SlateBuilder`): each subsystem `create()`s its elements at
  fixed paths; the builder lays out a fixed memory map. `SlateLayout::finalize()`
  clears the free list and initial-value scratch, making the structure immutable.
- **Run phase** (`Slate`): elements are accessed by token. References returned by
  `load()` are valid only for the current function scope — *"The references are
  invalidated as soon as a user gives up the flow of control."*

Two design choices make the bytes portable across machines and reboots:

- **No pointers are stored** in the model — only position-independent values — so
  the raw bytes are meaningful on a different computer or a different run (Rule
  #4 above). A `Slice` container is provided when you need to reference a span of
  an in-Slate array without a raw pointer.
- Because the layout is frozen and identical across units, a unit can **hash its
  layout and contents and compare them byte-for-byte against a peer**, and even
  compute the exact set of differences. The primitives (`Slate.h` / `Slate.cc`):
  - `get_shard_layout_hash(shard, Hash128&)` — hash of the *structure*.
  - `compute_hash(shard, UINT64&)` — content hash seeded by the layout hash;
    implemented over `digest_xxh128(buf, len, layout_hash)` then folded to 64
    bits (`Slate.cc:136`).
  - `compute_shard_deltas(shard, mem1, mem2, deltas, max_deltas)` — the exact set
    of differing regions, each a `shard_delta_t { size_t offset; B2c raw_data[2]; }`
    (`SlateMemory.h:27`).
  - `swap_shard_buffer(...)` — install a peer's buffer wholesale.

  This is the foundation of the agreement and recovery mechanisms below.

### Shards — per-element memory policy

State is tagged with a **memory policy** by placing each element in a **shard**.
There are **seven shards** (`num_slate_shard_t == 7`), created in
`FtRuntime::init_post_slate_build_early()`:

| Shard | Telemetered? | Cross-unit behavior |
|-------|:------------:|---------------------|
| `shard_static` | yes | **Constant / configuration** fixed at runtime. Policy is `slate_read_only` (`SlateLayout.cc:24`). |
| `shard_sync` | yes | **Replicated / voted** control state — kept bit-identical across all three strings. This is what `SlateSyncer` hashes and hotsyncs. |
| `shard_sync_no_telem` | no | Same as `sync`, but not emitted to telemetry. |
| `shard_nonsync` | yes | **Private / per-unit** bookkeeping — deliberately *not* shared (heap stats, local timers, non-synced commanding). |
| `shard_nonsync_no_telem` | no | Local, non-telemetered. |
| `shard_cyclic` | yes | **Per-cycle scratch** — replaced from a template every frame (see `roll_frame`). |
| `shard_cyclic_no_telem` | no | Cyclic, non-telemetered. |

These seven shards realize the **four memory policies** of the redundancy model,
each — except the constant policy — in a **telemetered and a non-telemetered
variant**:

- **Replicated / voted** → `shard_sync` · `shard_sync_no_telem`
- **Private / local** → `shard_nonsync` · `shard_nonsync_no_telem`
- **Per-cycle scratch** → `shard_cyclic` · `shard_cyclic_no_telem`
- **Constant** → `shard_static` (telemetered only — there is no non-telemetered
  constant shard, which is why the count is **seven, not eight**)

The cyclic shards are backed by `cyclic_mem_template` / `cyclic_mem_no_telem_template`
(`SlateMemory.h:127`), whose header comment reads *"This replaces the existing
cyclic shard memory at each frame."* `Slate::roll_frame()` (`Slate.cc:57`) swaps
the cyclic shard back to its template at end-of-cycle, invalidates all element
references (*"No memory references to elements inside the Slate are valid after
this point."*), and warns if leaked tokens are detected. All cyclic-shard work
must therefore complete before the end of `dispatch`.

**Permissions enforce the redundancy boundary at creation time.** In
`FtRuntime::init_pre_control_early()`:

```cpp
// Non-control (runtime/local) slates cannot create elements in the sync shard:
const slate_permission_t runtime_permission =
    slate_permission_deny(slate_permission_rwc, slate_permission_c_sync);
// The control slate can create sync elements, but NOT nonsync ones:
const slate_permission_t control_permission =
    slate_permission_deny(slate_permission_rwc, slate_permission_c_nonsync);
```

So only the control logic may create **replicated** (`sync`) state, and the
control logic is barred from creating **private** (`nonsync`) state. This draws a
hard line between "the shared truth all units must agree on" and "things each
unit keeps to itself."

---

## Why this is not shared memory

Each unit holds its **own private copy of the Slate in its own ordinary process
heap**. `SlateMemory` holds one `AlignedBuffer shard_table[num_slate_shard_t]`
allocated from the normal heap (`SlateMemory.h`). There is:

- **no `shm_open`**, no `mmap(MAP_SHARED)`, no cross-process shared pages for
  Slate data;
- **no memory-mapped file** used for IPC.

Units never read each other's memory. The **only** `mmap` anywhere in the tree
is in `DroneFtRuntime.cc`, where it maps **FPGA register banks**
(`/sys/devices/platform/amba_pl/.../mmap`) — memory-mapped *hardware I/O*, not
interprocess communication.

Consequently, **crossing any process boundary always means serializing the
relevant bytes and sending them as a network message.** This is a deliberate
isolation property: because units share nothing physically, a fault, a wild
write, or a crash in one unit cannot corrupt another's state — the only thing
that crosses the boundary is a well-formed message the receiver validates. There
is therefore **no "shared-memory path" and no fallback to switch to** — there is
exactly one inter-unit path, message passing, used uniformly.

Even **collocated** strings (`triple_string_colloc_ft_node`) and even a string
feeding its own data back into the pipeline go through the same messaging stack
via loopback. `FtRuntime::handle_local_share_read()` reads the node's own
local-share datagram off a `DataDgramChannel` and re-injects it into the
data-sharing pipeline *"to make it look like the data came from our node via the
node IO manager."* The mental model: **one process == one Slate in local memory;
crossing a process boundary always means a datagram.**

---

## Transport and naming

### Channels and connections (`vehicle/src/bullwinkle/all/io/`)

All IPC bytes travel over sockets wrapped by the `io/` channel abstraction.

- **`FdDgramChannel`** — *"A duplex DgramChannel backed by a datagram-oriented
  file descriptor. Data is read from the file descriptor into an internal
  buffer, at which point `read_sig` is emitted. Writes are similarly buffered
  until the underlying fd emits a write event, at which point they are flushed
  and `write_sig` is emitted."* (`FdDgramChannel.h`). It is buffered (a
  `BipBufferHeap` each direction, sized for ~2 datagrams at 1500 B by default),
  duplex, and re-openable after close via `assign_fd()`. Its `dgram_fd_type_t`
  enumerates the kinds of fd it speaks: `dgram_fd_non_socket`,
  `dgram_fd_device_file`, `dgram_fd_conn_socket` (connected, `recvfrom`),
  `dgram_fd_uconn_socket` (unconnected, sockaddrs), and `dgram_fd_tun` (TUN
  devices). A `set_low_latency()` mode forces immediate flush (at most one
  datagram per `select()`) for control-critical paths.

- **`UdpConnection`** — *"A DgramConnection which sends all datagrams to a UDP
  port. Once `connect()` has been called, UdpConnection will automatically open a
  new socket if it goes down for any reason. This class supports IPv4 and
  IPv6…"* (`UdpConnection.h`). Supports **multicast** (`multicast_write_bind`,
  `multicast_group_join/leave`, `multicast_recv_all`) and **broadcast**
  (`set_broadcast`), with an `ExponentialBackoffTimer` driving reconnection.
  This is the **primary cross-process / cross-computer transport.**

- **Stream/TCP** — `StreamConnection` (*"A duplex StreamChannel which may have
  intermittent connectivity"*), `TcpConnection` (auto-reconnecting),
  `TcpServer` (*"listens on a TCP port and spawns FdStreamChannels for each new
  connection"*), `TcpMultiServerConnection` (write-only multiplexing server),
  and `TcpP2PServerConnection` (single point-to-point connection at a time).
  Used mainly for **outbound telemetry destinations** and point-to-point links.

- **`Server`** (`Server.h`) — generic accept/prune connection-oriented server
  framework.

The whole stack is driven by an event loop:

- **`FdBag`** — *"A collection (or bag, if you will) of FdEventSinks with a suite
  of methods for dispatching them… any calls to the `select_*` method will
  dispatch active events on them through their FdEventSinks."* (`FdBag.h`). It is
  the epoll-based event loop (using `epoll_pwait2` for nanosecond resolution on
  kernels ≥ 5.11, falling back to `epoll_pwait`). It offers `select_absolute`,
  `select_relative`, `select_until_reads/writes`, and a non-blocking
  `select_once()` that is simply `select_absolute(nano_t_min)`.

- **`DeferredCallbackQueue`** — *"Thread-safe callback queue… useful for
  synchronizing interrupts like gRPC handlers with the rest of the
  single-threaded rocket world."* Callbacks run in strict FIFO on the event
  loop; an `eventfd` wakes the `FdBag`. This is how asynchronous gRPC completions
  are marshalled back onto the deterministic single-threaded control loop.

### Service directory / naming

Endpoints are never hardcoded. Code refers to **logical service names** resolved
at runtime to a concrete host/port/protocol via `service_directory().lookup()`,
which fills a `Service` whose `proto` field is `udp_proto` or `tcp_proto`. Two
concrete examples from `DroneFtRuntime.cc`:

```cpp
const std::string timestamp_service = "satgps1" + ident.string + "_timestamp";
FswAbortIfNot(service_directory().lookup(timestamp_service, service), false);
FswAbortIfNot(service.proto == udp_proto, false);
...
service_directory().lookup(Satellite::alert_buffer_output_service, alerts_service);
FswAbortIfNot(alerts_service.proto != udp_proto, false /* must be udp */);
```

A separate `node_directory()` maps the redundant units to network addresses, and
`verify_node_directory_segments()` validates that every node's IP lands in an
expected vehicle network segment:

```cpp
segments = { vehicle_network_segment_ground, vehicle_network_segment_rf,
             vehicle_network_segment_satellite,
             vehicle_network_segment_satellite_utility,
             vehicle_network_segment_satellite_payload };
FswAbortIfNot(verify_node_directory_segments(node_directory(), segments,
                                             true /* allow_192_168 */), false);
```

The topology can therefore be rearranged without touching the control logic.

---

## Communication style and message granularity

There are two distinct mechanisms, and **neither is a generic broker-style topic
bus**:

- **Within a unit**, components communicate by an **observer/signal pattern**.
  `Signal.h`: *"A Signal is a mechanism for communicating via function calls
  between disparate parts of a code base. A set of functions can be connected to
  a Signal, and when `emit()` is called, that set of functions will be executed.
  This allows for a single producer, multiple consumer paradigm."* Consumers
  `connect()` a `Slot`; producers `emit()`. It is publish/subscribe in spirit
  (`channel->read_sig`, `data_sharer->shared_data_sig`,
  `bootstrapper->full_sync_established_sig`), but delivery is direct in-process
  function calls — not a named-topic bus. (Lifetime is safe: a `SignalHandler`
  invalidates its slots on destruction, and emission is re-entrancy-locked.)

- **Between units**, distribution is **configuration-driven, not dynamically
  subscribed**: each sender is handed a static list of value-paths to share (a
  `sharer_config_v`), and each receiver knows where to deposit them. There is no
  runtime "subscribe to topic X" broker; the **path name is the topic-like
  identifier**, and commands/telemetry address elements by **name or hash**.

**Messages are batched, never one-value-per-message:**

- **Input sharing** packs a whole configured element list into one datagram,
  sized by `node_configs.get_node_output_buffer_size(...)`.
- **State recovery** transfers an **entire shard** at once (the `sync` shard,
  noted in `FtRuntime` as *"usually very large"*).
- **Telemetry** frames many channels together into BWP datagrams.
- **Commands** batch up to **50** element-sets atomically
  (`static_vector<multi_command_t, 50> multi_command_v`).

Granularity is **shard / element-set per message**, chosen for cache coherency
and network efficiency rather than fine-grained chatter.

---

## Keeping the three units in agreement

Four cooperating mechanisms make three independent processes behave as one.

### 1. Time alignment — `FtSync`

A distributed timing scheme — effectively a **phase-locked loop across units** —
keeps all three on the same control cadence. The member is declared as
*"The phase-synchronization system. This keeps all redundant strings of the
controller in a phase-locked loop."* (`FtRuntime.h:396`).

`FtSync::dispatch()` runs **first** every cycle, before anything else, because it
can set the EventLoop's control time:

```cpp
/*
 * Run FtSync first. This is necessary because it can set the
 * EventLoop's control time, so all other components MUST come
 * afterwards to ensure that they use a consistent control time.
 */
ft_sync->dispatch(0);
```

The whole control `dispatch()` *"return[s] `nano_t_min` always to ensure that
the EventLoop calls us back immediately. This allows FtSync to govern control
cycle timing."* (`FtRuntime.cc:272`). The control **period** itself is obtained
from `ft_sync->get_sync_period()` at init. One unit is the designated timing
initiator — `satfc1` *"should always be the time synchronization initiator"*,
wired in `DroneFtRuntime::create_bootstrapper` with
`FtBootstrapper::auto_time_sync`.

### 2. Input agreement and voting — `SlateSharer` + `FtSimpleDataSharer` + `SlateCombiner`

Each unit broadcasts the inputs it has gathered to the others over dedicated
datagram links arranged as a **ring** — each unit talks to a left and a right
neighbor — using a **two-pass share-then-reshare** exchange so data propagates
even if one link is degraded. `FtRuntime::create_input_sharing_system` builds
four UDP connections per unit:

```cpp
Handle<UdpConnection> left_share_conn;     // → left neighbor   (share pass)
Handle<UdpConnection> right_share_conn;    // → right neighbor  (share pass)
Handle<UdpConnection> left_reshare_conn;   // → left neighbor   (reshare pass)
Handle<UdpConnection> right_reshare_conn;  // → right neighbor  (reshare pass)
```

Each unit serializes its locally-acquired inputs with a `SlateSharerSender`
(configured by a `sharer_config_v` list of element paths) and `FtSimpleDataSharer`
exchanges them. Inbound input messages are **cryptographically signed and
verified** via a `Keychain`, so a unit cannot be fed forged inputs. On the
receive side (`BasicControl`), each peer's data lands in a per-string slate
(`TripleString<SlateBuilder> slate_shared` → `a`/`b`/`c`) via three
`SlateSharerReceiver`s.

Every unit then **votes per value across up to three sources** with
`SlateCombiner` (`SlateCombiner.h`):

> *"The output elements are either a 3/3, 2/2, 1/1 median of the source Slate
> values on an element-by-element basis depending on freshness, or a direct copy
> of all the values from the first fresh source. … If no inputs are fresh then no
> outputs will be updated."*

- **Freshness** is tracked per source by a monotonic `fresh_tok` element plus a
  cycle counter (`fresh_age_tok`) capped at `stale_threshold`; a source that
  hasn't updated within the threshold has its `connected_tok` cleared and is
  dropped from the vote. The default `stale_threshold = 1U` (one cycle) for
  time-synchronized use; it can be raised for slow, idempotent,
  non-time-synchronized inputs.
- A **`downselect`** mode exists for sets of values that must stay mutually
  consistent: *"to avoid 'tearing' messages that require inter-element coherency.
  If there is not a clear majority (two or more exactly equal sources), then the
  first connected source will always be used."* — i.e. it takes one connected
  source wholesale rather than mixing sources per value.

In `BasicControl`, `slate_combiner_control` combines the three `slate_shared`
inputs into `slate_control`. After this step, **all three units hold an
identical, agreed-upon input set.** This is the literal TMR input vote.

### 3. Identical computation, optional output voting

Because the logic is deterministic and the inputs are now identical, all three
units independently produce the same `sync`-shard state and the same outputs —
and they only run the control step once the supervisory layer reports they are
synchronized:

```cpp
if (bootstrapper->is_synced()) { ... control->dispatch_synced(); ... }
```

Telemetry from "muxed" groups must even produce **identical BWP framing across
strings** (`get_muxed_telemetry_groups`; the `telem_mux_strings` flag is OR'd
onto a flow when its group is in the muxed set).

In the strongest configurations (**triple-dual**), the **outputs are
additionally routed to a comparator and voted** before they take effect, instead
of being applied directly. The data sharer's output signal is left unconnected
from the channel manager precisely so the output-sync system can intercept it:

```cpp
if (!props.use_output_sync)
{
    data_sharer->shared_data_sig.connect(
        make_slot(*channel_manager, &FtChannelManager::read_input));
}
```

`use_output_sync` is true *"only [for] the triple-dual master and slave"*.

### 4. State recovery — `SlateSyncer` hotsync

Units **continuously checksum their replicated state and offer it to each
other.** A unit that reboots or drifts out of agreement can **pull the entire
`sync` shard from a healthy peer and swap it in wholesale**
(`swap_shard_buffer`), then rejoin — no cold start required. Each cycle:

- `dispatch_process()` applies an incoming hotsync (state recovery), run early;
- `dispatch_check()` detects desync before any control code runs;
- `dispatch_share()` hashes the `sync` shard and offers it to peers, run late.

Recovery transfers can be large, so `SlateSyncer` is given its **own dedicated
`FdBag`** so the bulk transfer is dispatched separately from normal I/O. When
`slate_syncer_synchronous_send` is set, the shard is flushed with a non-blocking
poll right after it is prepared so the bulk transfer overlaps with other
outbound traffic:

```cpp
slate_syncer_fd_bag->select_absolute(nano_t_min, fd_write_ev, time_used);
```

Whether recovery and transfer are even enabled is itself commandable: the policy
is *enabled* in `DroneFtRuntime` (`slate_syncer->enable_transfer()` +
`enable_hotsync()`), but the live gates `single_peer_hotsync_enabled` and
`force_disable_transfer` are **Slate elements read each cycle**, so the policy
can be controlled in flight.

A small supervisory state machine, `FtBootstrapper`, drives each unit from "just
started / out of sync" through to "fully synchronized and contributing"
(`is_synced()`), and emits `full_sync_established_sig` when it gets there.

### Node types — what each role turns on (`FtNodeProperties`)

The five node roles and the synchronization features each enables are derived
mechanically in `FtNodeProperties` (`FtRuntime.h:115`):

| `ft_node_type_t` | input sync | output sync | slate syncer | firmware comm |
|------------------|:----------:|:-----------:|:------------:|:-------------:|
| `triple_string_ft_node` | ✓ | – | ✓ | ✓ |
| `triple_string_colloc_ft_node` | ✓ | – | ✓ | ✓ (collocated on one computer) |
| `triple_string_guest_ft_node` | ✓ | – | ✓ | – (network-only "VM"-like host) |
| `triple_dual_master_ft_node` | ✓ | ✓ | – | – |
| `triple_dual_slave_ft_node` | – | ✓ | – | – |

The exact derivation (verbatim):

```cpp
explicit FtNodeProperties(const ft_node_type_t ft_node_type)
  : use_input_sync (ft_node_type != triple_dual_slave_ft_node),
    use_output_sync(ft_node_type == triple_dual_master_ft_node ||
                    ft_node_type == triple_dual_slave_ft_node),
    use_slate_syncer(ft_node_type != triple_dual_master_ft_node &&
                     ft_node_type != triple_dual_slave_ft_node),
    use_firmware_comm(ft_node_type == triple_string_ft_node ||
                      ft_node_type == triple_string_colloc_ft_node),
    use_ftrace_trap(ft_node_type != triple_dual_slave_ft_node &&
                    ft_node_type != triple_string_colloc_ft_node) {}
```

The guest type is described in the source as one that *"does not run FirmwareComm
and can be imagined as a 'Virtual Machine' on a host, that doesn't particularly
care about which host it is on."* Drone flight computers run as
`triple_string_ft_node` (or `_guest` when they have no FirmwareComm).

---

## The control cycle

Each tick runs the same sequence on every unit, structured as three phases in
`FtRuntime::dispatch()` → `dispatch_nonsynced_early()` / `dispatch_synced()` /
`dispatch_nonsynced_late()`:

```
EARLY (per string, not yet voted):
  1. FtSync.dispatch(0)            → agree on control time (sets EventLoop time)
  2. control->start_cycle(...)
  3. SlateSyncer.dispatch_process → apply any inbound hotsync (state recovery)
  4. FirmwareComm write/read/update → exchange with FPGA/ADC hardware (DMA)
  5. adc_scaler / timestamp_gatherer → scale inputs, stamp local time
  6. gnd_cmd_dispatcher_nonsynced  → apply private (non-synced) commands
  7. slate_sender_local.share      → broadcast my inputs to peers   ── UDP ──►
  8. data_sharer.dispatch          → exchange (share+reshare) + vote ◄── UDP ──
       (non-sharing nodes instead fswsleep(ds_parallel_sleep_time))
  9. SlateSyncer.dispatch_check    → desync detection
 10. bootstrapper.dispatch         → advance sync state machine
 11. FtSync.send_syncs (cond.)     → emit time-sync beacons         ── UDP ──►

SYNCED (only if bootstrapper->is_synced() — identical on all strings):
 12. control->dispatch_synced      → state machine, alarms, control law, outputs

LATE (per string):
 13. upkeep / heap stats / ftrace trap
 14. telem_relay.dispatch          → frame + emit telemetry      ── UDP/TCP ──►
 15. SlateSyncer.dispatch_share    → hash sync shard, offer hotsync to peers
       (+ optional non-blocking "select once" to start the big sync-shard send)
 16. slate.roll_frame              → wipe cyclic shard for next cycle
 17. channel_manager.flush_inputs  (conditional)
```

Units that don't gather inputs themselves (e.g. those without input sync) still
hold the cadence by **sleeping for the equivalent amount of time** the sharing
units spend exchanging data, so the whole group stays phase-aligned regardless
of each unit's role:

```cpp
// On systems that do not use input sync, we instead sleep for the duration of
// share time and reshare time to keep them in phase with their counterparts…
ds_parallel_sleep_time = share_time + reshare_time;
...
fswsleep(ds_parallel_sleep_time);
```

---

## Commands and telemetry

### Commands in

Inbound commands are deframed, time-filtered, deduplicated, authenticated, and
applied as changes to named values. The driver is a `SlateCommandInterface` —
*"A runtime interface to Slate that allows elements to be set by name or hash.
This is useful for commanding."* It exposes typed setters in both addressing
modes: `set_int_by_name` / `set_int_by_hash`, `set_fp_by_name` /
`set_fp_by_hash`, etc., keyed by an `external_command_name_hash_t`.

A single command can set **dozens of values atomically**:

```cpp
// "A vector of multi commands. We should never accept more than 50 commands."
typedef static_vector<multi_command_t, 50> multi_command_v;
...
// "Set multiple elements by hash. This command is atomic: if any individual set
//  is rejected … all sets will be rolled back."
bool set_multi_by_hash(multi_command_v &cmds, bool &success) RUNTIME;
```

Authentication is by **Ed25519 signature** verification, with two keystores —
`operator_keystore` (operator) and `command_auth_keystore` — and a `require_signed_command`
gate (`CommandQueueClient.h`). Name↔hash↔id resolution is backed by a
`SymbolTable`: *"A map of strings to integers and back based off of the contents
of a data file … maps in either direction, and has an optional fallback
mechanism."*

There are **two command paths**:

- a **replicated** one in `BasicControl` (`ground_cmd_dispatcher_synced`) that
  writes the `sync` shard so all units stay in agreement, and
- a **private** one in `FtRuntime` (`gnd_cmd_dispatcher_nonsynced`) that writes a
  single unit's `nonsync` shard.

The corresponding state elements live in the matching shards — e.g.
`CommandQueueClient` creates `accepted_sequence_number` in `shard_sync`, while
`CommandPump` creates `command_pump.last_accepted_seq_num` in `shard_nonsync`.

The link to ground is carried over **gRPC**. The vehicle is a client of a command
service's `PumpQueue()` RPC — *"Trades command feedback for the next command to
send, if any."* (`CommandQueueClient.h`). A `GrpcPoller` drives the polling
(defaults: `grpc_timeout = billion/2` = 500 ms, `grpc_polling_period` = 1 hour,
retry backoff 1 s → 32 s, jittered to avoid a thundering herd), and endpoints can
be static (`StaticGrpcEndpoint`) or DNS/SRV-resolved with up to `MAX_TARGETS = 6`
failover targets (`DynamicGrpcEndpoint`). Because polling once per hour is too
slow when a link comes up, an **`AcquisitionOfSignalMonitor`** *"Monitors when
the bidirectional comms flag goes high and emits a signal on acquisition of
signal. Useful to trigger slow polling gRPCs when bidirectional comms is
observed."* — it fires `retry_grpc_sig` to kick the poller the moment a link is
acquired (AOS threshold = 1 s).

### Telemetry out

`TelemetryRelay` packages the Slate into **framed, rate-limited** streams. It
frames Slate data into **BWP** datagrams with `BwpWriter` (packet size
`bwp_datagram_packet_len`; default flow buffers hold ~10 packets), organized into
**flows** and **groups** (`telem_id_t`, `telem_group_t`), and sends them to
destinations resolved through the service directory over UDP or TCP.

A **byte-quota throttle** bounds the outbound rate per flow, configured by a
`bandwidth <burst> <recharge>` directive that sets `bandwidth_burst` (bytes) and
`bandwidth_recharge` (refill rate):

```cpp
relay_flow->set_bandwidth(flow.bandwidth_burst, flow.bandwidth_recharge);
```

Selected flows are **redirected to local consumers** instead of being
transmitted — e.g. the alert buffer in
`DroneFtRuntime::create_local_telemetry_connections`, via `redirect_service`.
Groups marked "muxed" (`get_muxed_groups`) carry the `telem_mux_strings` flag so
their frames are produced identically on every string — a cross-check that the
redundant units really did compute the same thing.

---

## In one sentence

Three physically independent computers each keep a private, pointer-free,
fixed-layout copy of one named model of all vehicle state (the **Slate**); they
share nothing in memory and communicate only by authenticated, batched network
datagrams; they stay locked to a common cadence (**FtSync** PLL), reach agreement
by ring-exchanging and median-voting their inputs (**FtSimpleDataSharer** +
**SlateCombiner**) and optionally their outputs (triple-dual), continuously
checksum and wholesale-recover their replicated `sync` shard from one another
(**SlateSyncer** hotsync, driven by the **FtBootstrapper** state machine), and
expose unified command and telemetry interfaces (**SlateCommandInterface**,
**TelemetryRelay**) — so the vehicle keeps flying through the loss of any single
unit.

---

## Appendix: where to look in the source

| Concern | Primary files |
|---------|---------------|
| Data model, rules, shards, hashing | `bullwinkle/all/Slate.{h,cc}`, `SlateBuilder.{h,cc}`, `SlateMemory.h`, `SlateLayout.{h,cc}`, `slate_tokens.{h,cc}`, `slate_accessor.h` |
| Input voting | `bullwinkle/all/SlateCombiner.{h,cc}` |
| In-process pub/sub | `bullwinkle/all/Signal.h` |
| Transport / channels | `bullwinkle/all/io/{FdDgramChannel,UdpConnection,FdStreamChannel,TcpConnection,TcpServer,TcpMultiServerConnection,StreamConnection}.{h,cc}`, `Server.{h,cc}` |
| Event loop / async | `bullwinkle/all/FdBag.{h,cc}`, `bullwinkle/all/async/DeferredCallbackQueue.{h,cc}` |
| FT runtime, cycle, node types, time/input/output sync, recovery | `flight/common/all/FtRuntime.{h,cc}`, `flight/common/all/BasicControl.{h,cc}` |
| Satellite specialization, mmap'd FPGA, service/node directory, bootstrapper | `DroneFtRuntime.{h,cc}`, `flight/sat/all/flight-computer/DroneGnc{Runtime,Control}.{h,cc}` |
| Commands | `SlateCommandInterface.{h,cc}`, `CommandQueueClient.{h,cc}`, `CommandPump.{h,cc}`, `CommandSender.{h,cc}`, `SymbolTable.{h,cc}`, `AcquisitionOfSignalMonitor.{h,cc}` |
| Ground-link gRPC | `gRPC_endpoint.{h,cc}`, `gRPC_util.{h,cc}`, `GrpcClient.h`, `GrpcPoller.h` |
| Telemetry | `TelemetryRelay.{h,cc}` |

> Several implementation headers the runtime *uses* are referenced from
> construction/dispatch sites but are not present in this slice of the tree —
> notably `SlateSharer`/`SlateSharerSender`/`SlateSharerReceiver`, `SlateSyncer`,
> `FtSync`, `FtSimpleDataSharer`, `FtBootstrapper`, `NodeIoManager`, and
> `FtChannelManager`. Their behavior here is reconstructed from how
> `FtRuntime`/`BasicControl`/`DroneFtRuntime` construct and drive them.
