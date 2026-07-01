# Codon Flight Software — Implementation Roadmap

> **What this is.** The repository today is a *slice*: the Slate data model, a few
> transport/utility leaf classes, the telemetry relay, and the runtime/control
> scaffolding — but ~172 of the headers it `#include`s are absent, there is no
> entry point, no build target for the flight code, no hardware layer, no config,
> and no tests. This roadmap is the plan to fill those gaps into a system that
> **builds, runs in software-in-the-loop (SIL), and ultimately flies on a drone.**
>
> **How the gaps were inferred.** Every missing component's contract below was
> reverse-engineered from how the *present* code constructs and calls it (the
> `#include`s that don't resolve), then checked against the architecture in
> [`SYSTEM_DESIGN.md`](SYSTEM_DESIGN.md). Where a contract only appears in
> `SYSTEM_DESIGN.md` and not in code, it is flagged as design-only.
>
> **Two tracks.** **Track A** makes the existing (satellite flight-computer)
> framework build and run in SIL — this proves the triple-modular-redundant IPC
> end to end. **Track B** pivots the *vehicle domain* from satellite to drone. The
> `Starlink→Drone` rename so far was textual; the GNC/power/RF/time code is still
> spacecraft (see §6).
>
> **Coding standard.** All new code follows the **`data-oriented-design`** skill
> (`~/claude-skills/skills/data-oriented-design/`). §1 makes its rules binding for
> this repo. This is not optional styling — the control loop is hard-real-time and
> the Slate is a pointer-free, fixed-layout, byte-comparable model; the skill's
> doctrine is what keeps both true.

---

## 1. Coding standards (binding — from the `data-oriented-design` skill)

Every milestone's code MUST satisfy these. They are the repo's existing invariants
(Slate rules, `RUNTIME`/`INFRASTRUCTURE` annotations, `Handle`, `Signal`,
`static_vector`, `B2`/`B2c`, arenas) restated as enforceable rules. Cite the skill
file in parentheses.

### 1.1 The optimization order (never skip a rung) — *skill SKILL.md*
Algorithm/complexity → **data layout** → ownership/lifetime → memory traffic →
branch behavior → vectorization/batching → codegen. Most of this code is rung 2–3:
the Slate *is* the layout; get it dense and pointer-free first.

### 1.2 Hard rules
- **All state lives in the Slate. No private state in objects.** (Slate Rule #1;
  enforced by `RUNTIME`/`INFRASTRUCTURE` macros from the missing `runtime.h`.) A
  type that holds control state outside the Slate cannot be telemetered, voted,
  hotsynced, or recovered — it breaks the redundancy model.
- **No pointers stored in the Slate — position-independent values only.** (Slate
  Rule #4; *bit_packing_and_swar.md*, *data_layout.md*.) Use `slate_element_t`
  ids/indices and `B2`/`B2c` spans, never raw or smart pointers, in Slate data.
- **No hidden allocation, blocking, throwing, or dynamic dispatch in a `RUNTIME`
  (hot-path) method.** (*SKILL.md* hard rules; *control_flow_and_branchless.md*.)
  The per-cycle `dispatch_*` chain runs at the control rate; allocation or a
  syscall there is a deadline miss. Use arenas/`static_vector`/fixed capacity.
- **Fixed capacity, no resize/relocate, for anything on the hot path.**
  (*data_layout.md*; mirrors `fast_map`'s "never resize → stable addresses".)
  `static_vector<T,N>` over `std::vector`; bounded queues over unbounded
  (*algorithms_and_structures.md* "every queue is bounded").
- **SoA / hot-cold split for scanned data; align to the access unit.**
  (*data_layout.md*.) Telemetry/voting scan one field across many elements — lay
  those out as arrays; `alignas(64)` SIMD/cache-line-hot structures; `alignas(128)`
  contended words to kill false sharing.
- **Branchless on unpredictable, data-dependent hot branches; explicit failure
  signals.** (*control_flow_and_branchless.md*.) Prefer `Result`/`[[nodiscard]] bool`
  (the repo's `FswAbortIfNot`/`result_t` idiom) over exceptions; no exceptions for
  control flow.
- **Ownership is visible in the type.** (*SKILL.md*; the repo's `Handle<T>` =
  shared, `SlateBuilder` value = build handle, `B2c` = borrowed const span,
  arena/scratch = phase-scoped.) No ambiguous ownership through globals.
- **Every "faster"/"fits the deadline" claim carries a measurement.**
  (*measurement_and_verification.md*.) Hot paths get a benchmark + a worst-case
  execution-time (WCET) number, not a vibe.

### 1.3 Measurement & verification gate (per component)
Pipe build/test output to a log; `scripts/check.sh` must be green. For hot-path
code: `perf stat` (cache-misses, branch-misses, cycles), `objdump -d` to confirm
no hidden calls/allocs in `RUNTIME` methods, and a microbench vs a scalar baseline.
For the control loop: a WCET measurement against the control period
(`FtSync::get_sync_period()`). See *measurement_and_verification.md* for the toolset.

### 1.4 What "done" means for a component
Builds under its BUILD target · its `RUNTIME` methods provably allocate/throw/block
nothing (audited) · ownership expressed in types · a unit test exercises empty/full/
error paths · for hot code, a benchmark + WCET · consistent with the
`SYSTEM_DESIGN.md` mechanism it implements · **its entry in
[`docs/implementation.md`](docs/implementation.md) is added/updated** (contract,
test, verification, any flags).

---

## 2. Layered dependency order (the build order)

Bottom-up; each layer compiles only after the one below. Milestones (§4) follow
this order. "(sat)" marks layers with satellite-specific content needing a drone
replacement (Track B, §6).

```
L0  Core primitives        fsw, drone_types, fswtime, util, Hash128/xxh, static_vector,
                           B2/B2c (real), runtime.h macros, enum/auto_enum + *.enum.h, BUILD
L1  Slate completion       SlateElement, SlatePathMap, SlateBuilderStore, slate_info,
                           EnumRegistry, ReflectionManager, SlateDump
L2  Event loop + transport EventLoop/EventSource/EventList/Periodic, Clock, FdEventSink,
                           io/Channel·DgramChannel·StreamChannel·DgramConnection·forks·
                           Data*Channel·NullDgramChannel, BipBuffer, DataQueue, net/sock/
                           multicast utils, Crc, Reconnector, ExponentialBackoffTimer, ServerFd
L3  Naming + config        Configs, ServiceDirectory (service.h, vehicle_network.h),
                           NodeIdentifier, ft/FtNodeConfig
L4  Telemetry              BwpFramer, BwpWriter, BwpChannelWriter, ByteQuotaFramer,
                           TelemetryFlowInfo, TelemetryRelayFlow, TelemetryWriter,
                           *TelemetryTask, DeviceTelemetryFactory, GroundTelemetryRelay
L5  Commanding + security  ExternalCommandDispatcher + handlers, CommandTable,
                           external_command_util, CommonCommandFilter,
                           ExternalCommandFilterCommon, Keychain, Ed25519Crypto (+proto),
                           hsm/* (SIL stub), CommandPayload (sat)
L6  FT mechanisms (TMR)    FtSync, FtSimpleDataSharer, NodeIoManager, ft/FtChannelManager,
                           SlateSharer{Sender,Receiver,Manager}, SlateSyncer, FtBootstrapper,
                           TimestampGatherer, TimestampSynchronizer, time_slave_preload,
                           AdcScaler, FtraceTrap
L7  Control + GNC + entry  ControlInterface/BasicControl (present), ControlState, ControlTask,
                           StateMachine (present), GncController, GncComponentFactory,
                           StateRegistrySlateInterface, DroneControl (sat), main()
L8  Hardware (HAL)  (sat)  FirmwareComm, FirmwareCommAdcInit, RealTimeClockInterface,
                           WatchdogHeartbeat, PpsManager, SwiftPpsInterface,
                           PowerConverterInterface, TtcRuntime
L9  Vehicle domain  (sat)  DroneGncControl law, DroneRegisterFlightComputerGnc,
                           ColaBurnMonitor, satellite_constants/rev enums → DRONE equivalents
L10 Production hardening    RTOS/scheduling/mlockall, signed boot, OTA, HIL, soak, real keys
```

---

## 3. Component inventory (inferred contracts, by layer)

Condensed; each entry: **responsibility** — *key inferred interface* — `SYSTEM_DESIGN`
tie-in. Full call-site evidence exists for every signature (reverse-engineered from
construction/dispatch sites in the present code). Path is the missing include path.

### L0 — Core primitives
- **`core/fsw.h`** — assertion / early-return / debug-print vocabulary used in every
  function. *`FswAbortIfNot(cond,ret)`, `FswAbortIf`, `FswMsgAbortIfNot`, typed
  `FswAbortIfNeq*`/`FswAbortOutsideRange*`, predicate `FswIf*`, `FswAssert`/
  `FswDebugAssert` (compiled out in flight), `dbnprintf`/`dbvnprintf`, `FswPrefix`,
  `report_abort`, `FswStackFrame::get_current_stack_frame()`.* — infrastructure for
  "validate every message, fail closed" isolation.
- **`core/drone_types.h`** — fixed-width typedefs + copy-suppression. *`UINT8..64`,
  `INT8..64`, `uint`, `FSW_DISALLOW_COPY_AND_ASSIGN`.* — pointer-free fixed layout.
- **`core/fswtime.h`** — time vocabulary. *`nano_t` (signed 64-bit ns), `nano_t_min`/
  `nano_t_max`, `get_rel_time()`, `fswsleep(nano_t)`.* — control cadence (Time
  alignment).
- **`core/util.h`** — `MonotonicPool` (build-phase arena: `allocate(size,align)`,
  `release()`), `join(str_v)`, container typedefs (`str_v`, `str_s`, …). — arena
  discipline (*data_layout.md*).
- **`hash/Hash128.h`** — `struct Hash128 { UINT64 u64[2]; }`. **`hash/xxh.h`** —
  `Hash128 digest_xxh128(buf,len,seed)`. — byte-for-byte agreement & recovery
  (Build phase / state recovery).
- **`static_vector.h`** — `static_vector<T,N>` (inline, no heap): `push_back`,
  `size`, `operator[]`. — bounded TMR source set / 50-element command batch.
- **`runtime.h`** — `RUNTIME`/`INFRASTRUCTURE`/`RUNTIME_SIGNAL`/`HOTSYNC_EXEMPT`
  annotation macros the hotsync analyzer reads. — enforces the deterministic,
  allocation-free control loop.
- **`B2.h`/`B2c.h`** — **currently 0-byte stubs**; real types are byte spans.
  `B2c` const span: `B2c(const void*,size_t)`, `.buf()`, `.len()`. `B2` mutable. —
  unit of raw shard transfer/diff (`compute_shard_deltas`, `swap_shard_buffer`).
- **`enum/auto_enum.h` + generated `*.enum.h`** (`slate_shard_t`, `slate_elem_access_t`,
  `slate_subsystem_id_t`, `telem_group_t`, `ctask_state_t`, `hsm_type_t`,
  satellite revs, `calibration_status_t`) — each an enum + paired `_sym`
  `SymbolTable` + count sentinel (`num_slate_shard_t==7`). — the shard policy model
  and enum/telemetry metadata. **Needs a codegen step** (these are generated).

### L1 — Slate completion
- **`SlateElement.h`** — `struct SlateElementMetadata{type_id, shard, value_offset,
  value_size, access_policy, subsystem_id, is_view}`. — frozen layout record.
- **`SlatePathMap.h`** — ordered path→metadata map with stable id↔iterator
  (`insert`, `find`, `id_to_iterator`, `iterator_to_id`). — hierarchical path
  addressing → ids.
- **`SlateBuilderStore.h`** — `SlateBuilderStoreInterface` backing the builder:
  `slate_layout()`, `get_permission()`, `sub_slate(...)`, `allocate_element(...)`,
  `bind_read/write`, `build()/finalize()`, factory `CreateSlateBuilderStore()`. —
  build vs run split + permission/redundancy boundary.
- **`slate_info.h`** (also `core/slate_info.h`) — type-trait registry `slate_info<T>`
  (`is_valid/size/alignment/construct/from_mem/copy/type_id`), packed-id helpers
  (`slate_id_breakdown/buildup`), `slate_permission_t` + `slate_can_*`, validators
  (`SlateTypedValidator<T>`, `SlateAccessor<T>::store`), `result_t`. — uniform
  storage/voting/hashing foundation.
- **`EnumRegistry.h`** — `register_auto_enum<E>`, `register_enum`, `get_registered_enum`,
  `is_finalized` (held via `Handle`). — command name↔id + telemetry metadata.
- **`ReflectionManager.h`** — runtime reflection slots (`get_num_local_reflect_slots`,
  `dispatch`, static `create(num_slots, slate, shard, prefix, cmd_iface, out)`). —
  local introspection of `nonsync` state.
- **`slate_dump/SlateDump.h`** — periodic Slate snapshot to file (`dispatch`,
  `set_synced`). — diagnostics.

### L2 — Event loop + transport
- **`EventLoop.h`** — control-cycle scheduler + authoritative control time;
  `EventSource` base (`nano_t dispatch(nano_t)`, return `nano_t_min` ⇒ immediate
  re-dispatch), `EventList`, `Periodic`, `clock`, `control_time()`, `fds` (FdBag). —
  cadence + FtSync sets control time (Time alignment, control cycle).
- **`Clock`** — `nano_t` time source (`now()`/`get_rel_time`). — time alignment.
- **`FdEventSink.h`** — owns one fd; `add_events(mask, slot)`/`remove_events`,
  handler slot `Signal<bool, FdEventSink&, FdEvent&>`, `get_fd`, `close`. Created by
  `FdBag::fd(AutoFd)`. — epoll leaf of the event loop.
- **`io/Channel.h`** — duplex endpoint base; `write_sig`/`close_sig` =
  `Signal<bool,Channel&>`, `get_dataframe(size_t)→DataFrame&`,
  `commit_dataframe`, `close/clear`, protected `channel_*` hooks. — the `io/`
  substrate ("crossing a boundary == a datagram/stream").
- **`io/DgramChannel.h`** — `:Channel`, message-boundary; `read_sig=Signal<bool,
  DgramChannel&>`, `peek_dgram()→B2c`, `dgrams_avail`, `get_dgram().pop()`. —
  datagram family (ring + telemetry).
- **`io/StreamChannel.h`** — `:Channel`, byte stream; `read_sig=Signal<bool,
  StreamChannel&>`, `get_data()→B2c`, `pop_front`. — TCP/stream family.
- **`io/DgramConnection.h`** — `:DgramChannel` + `is_connected`/`disconnect`/
  `connect_sig` (auto-reconnect). — primary cross-unit transport.
- **`io/{Dgram,Stream}ChannelFork.h`** — write-side tee/multiplexer (`fork(channel,
  bool)`); back `TcpMultiServerConnection`. — multiplexing fan-out server.
- **`io/AnyDgramConnection.h`** — concrete UDP/local dgram conn: `AnyDgramConnection
  (EventList&, FdBag&[, num_dgrams, max_len])`, `open(host,port)`. — telemetry
  destination behind BWP writer.
- **`io/DataDgramChannel.h` / `io/DataStreamChannel.h`** — in-memory loopback
  channels (`DataDgramChannel(min_dgrams, max_len)` with `read_sig`/`write_sig`). —
  local-consumer redirection + collocated self-loopback ("Why this is not shared
  memory").
- **`io/NullDgramChannel.h`** — `/dev/null` dgram sink. — disable a destination
  without special-casing.
- **`io/TcpServerFd.h`** / **`ServerFd.h`** — per-connection objects for the `Server`
  accept/prune framework (`close_sig=Signal<bool,ServerFd&>`, `dispatch`, `accept`). —
  connection-oriented server.
- **`BipBuffer.h`** (`BipBufferHeap`) — bipartite ring for dgram framing
  (`get_dataframe`, `commit_dataframe`, `pop_dgram`, `dgrams_avail`). **`DataQueue.h`**
  (`DataQueueHeap`) — byte FIFO for streams. — channel backing stores.
- **`Reconnector.h` / `ExponentialBackoffTimer.h`** — reconnect drivers (backoff
  1 s→32 s jittered; `timer_sig`, `connect_sig`). — auto-reconnect.
- **`net.h`/`sock_util.h`/`multicast_utils.h`/`net_multicast.h`/`net_interface_utils.h`/
  `Crc.h`** — protocol constants (`udp_proto`/`tcp_proto`, `fd_*_ev`), addr/socket
  syscall wrappers (`fsw_recvfrom/sendto`, keepalive, bufsizes), multicast group ops,
  CRC. — transport substrate.

### L3 — Naming + config
- **`Configs.h`** — config-file finder `config_file(name,out[,report])`; passed by
  const-ref everywhere. — configuration-driven distribution.
- **`service.h`/`vehicle_network.h`** (ServiceDirectory) — `service_directory().lookup
  (name, Service&)` (`Service{proto∈{udp,tcp}}`); `node_directory()`;
  `verify_node_directory_segments(dir, segments, allow_192_168)` over
  `vehicle_network_segment_*`. — service directory / naming, segment validation.
- **`NodeIdentifier.h`** — `NodeIdentity{role, role_inst, string, node_name}`,
  `node_id_t`; `shared_slate_name = role_inst + string` (`satfc1a/b/c`). — redundancy
  identity / per-string slate naming.
- **`ft/FtNodeConfig.h`** — `FtNodeConfigList`: `parse(configs,"node_mgr")`, `empty`,
  `get_node_output_buffer_size(node, size&)`. — topology/buffer config for the
  sharing ring.

### L4 — Telemetry
- **`io/BwpFramer.h` / `BwpWriter.h` / `io/BwpChannelWriter.h`** — BWP packetizer +
  abstract sink (`Handle<BwpWriter>`) + concrete writer over a `Channel`
  (`assign_channel(...)`, pull model via `write_sig`). Const `bwp_datagram_packet_len`/
  `bwp_mtu`. — "frames Slate data into BWP datagrams" (Telemetry out).
- **`io/ByteQuotaFramer.h`** — rate-limited BWP writer: `ByteQuotaFramer(conn,
  max_quota, recharge_bits_per_sec, signal_threshold)`, `init(slate,"framer_dgrams")`. —
  the `bandwidth <burst> <recharge>` byte-quota throttle.
- **`TelemetryFlowInfo.h`** — parsed flow descriptor (`id, group, type, hosts[],
  bandwidth_burst/recharge, idle_timeout, buf_size, static_flags`incl.
  `telem_mux_strings`, `num_alt_destinations`). — flow/group config (configuration-
  driven distribution).
- **`TelemetryRelayFlow.h`** — per-flow engine (`TelemetryRelayFlow(clock,id,group,
  type,enabled_tok,alt_dest_tok,node_src,buf_size,flags)`, `set_bandwidth`,
  `add_connection`, `dispatch`; is-a `TelemetryConsumer`). — flows framing/rate-limit/
  transmit.
- **`TelemetryWriter.h`** (+`TelemetryConsumer`) — producer handle (`assign_consumer`)
  filled by `claim(id,...,writer)`. — producer↔flow coupling.
- **`SlateTelemetryTask.h` / `StreamChannelTelemetryTask.h` / `DgramChannelTelemetryTask.h`
  / `DeviceTelemetryFactory.h`** — producers (`:TelemetryTask`, `dispatch(control_time,
  telem_time)`) walking the Slate / draining channels; factory builds per-device
  producers. — telemetering the named model; muxed-group cross-check.
- **`GroundTelemetryRelay.h`** (+`GroundNumericFlow`) — ground-link numeric flow
  (`add_slate_element(token)`); used by `GrpcClient`. — ground-link telemetry.

### L5 — Commanding + security
- **`ExternalCommandDispatcher.h`** — inbound engine: deframe→time-filter→cmd-filter→
  arm→dispatch chain; `config_params_t(dedup_capacity, dedup_expiration,
  proxy_watchdog_*)`, `init(slate, role, shard, inputs, deframer, time_filter,
  cmd_filter, armer, handlers, outputs)`, `dispatch() RUNTIME`,
  `get_watchdog_all_disconnect_sig()`. **Two instances: synced→`shard_sync`,
  nonsynced→`shard_nonsync`.** — the two command paths (Commands in).
- **`ExternalCommand{Slate,MultiSlate,Gnc,Reflection,StateMachine}Handler.h`** —
  handler chain: set-by-name/hash → atomic multi-set (50, rollback) → GNC registry →
  reflection slots → state-machine transitions. — command application.
- **`CommandTable.h`** — name/index↔`vehicle_cmd_t` (`init(configs)`, `lookup`,
  `populate_enums`). **`external_command_util.h`** — `external_command_name_hash_t`,
  `ext_cmd_handler_v`, default channel helpers, Null filter/armer/deframer. —
  command vocabulary.
- **`CommonCommandFilter.h` / `ExternalCommandFilterCommon.h`** — config-driven
  validate/authorize filter + adapter to the dispatcher's `ExternalCommandFilter`
  interface (`init_storage(configs, slate, shard, enable)`, full `init(... CommandTable,
  StateRegistry, allow-vars, prefixes, alert)`). — authentication/validation stage.
- **`Keychain.h`** — signing/verification keys for inter-unit datagrams (held via
  `Handle`, passed to `init_pre_slate_build` and `FtSimpleDataSharer::init`). —
  signed input sharing ("cannot be fed forged inputs").
- **`common/Ed25519Crypto.h`** — `Ed25519KeyStore<T>` (`init(slate)`, `unwrap(out,
  ephemeral_id, signed)`); two-layer NOC + command-auth verification. **(sat key
  hierarchy.)** — Ed25519 command auth.
- **`hsm/SslPrivateKeyMethod.h` (+Cmrt/Stsafe/TrustZone)** — HSM-backed mTLS key ops
  for the gRPC client identity (`GrpcAuthConfig.key_handler`, `hsm_type_t`). — ground
  link security. **SIL: software-key stub; flight: real HSM per board.**
- **`fleet_client/command/CommandPayload.h`** — `from_protobuf`, `write_message(msg,
  alias&)`. **(sat fleet path.)** — unit of work pulled over `PumpQueue()`.
- **Generated gRPC/proto** — `command_queue/service.{pb,grpc.pb}.h`,
  `security/{signed_data,tbs_command}.pb.h` (`PumpQueue` RPC, `PumpQueueRequest/Response`,
  `TBSCommand`/`SignedData`). **No `.proto` files exist — must be authored + codegen
  wired.** **(sat fleet API.)**

### L6 — FT mechanisms (the TMR core) — *full contracts; this is the heart*
- **`FtSync.h`** — distributed phase-lock PLL; **owns/sets the EventLoop control
  time**. `FtSync(EventLoop&, EventList&, bool auto_send_syncs)`; `init(slate_local,
  configs, cmd)`, `populate_enums`, `get_sync_period()→nano_t` (the control period
  source), `add_extra_fd_bag(Handle<FdBag>)`, `dispatch(0)` (runs FIRST),
  `send_syncs()`, `reset_counters()`. Slots: `bootstrapper.time_sync_established_sig→
  reset_counters`, `data_sharer.reshare_phase_sig→send_syncs`. — **Mechanism #1**;
  cycle steps 1 & 11. *Design check: confirms "FtSync runs first and sets control
  time", and that the control period is FtSync's, both as SYSTEM_DESIGN states.*
- **`FtSimpleDataSharer.h`** — ring share/reshare of local inputs; signs/verifies via
  Keychain. `FtSimpleDataSharer(Clock&, NodeIoManager::data_sig, bool overlap)`;
  static `create_sharing_connections(node_configs, ident, service_directory(),
  upkeep, fds_share, fds_reshare, left/right_share_conn, left/right_reshare_conn)`;
  `init(slate_local, role_inst, string[0], node_configs, keychain, fds_input,
  fds_share, fds_reshare, 4×conn, share_time, reshare_time, compute_crc)`;
  `dispatch(control_time)`. Signals: `shared_data_sig→FtChannelManager::read_input`
  (only if `!use_output_sync`), `reshare_phase_sig`. — **Mechanism #2** send/exchange;
  cycle step 8. *Design check: confirms the four-UDP ring + two-pass + Keychain
  signing exactly as documented.*
- **`NodeIoManager.h`** — opens input-gathering connections; emits `data_sig
  (node_id, data)`. `init(slate.sub_slate("node_io"), node_configs, ident, upkeep,
  fds_input, eloop.fds)`, `init_output_sources(channel_manager)` (if `!use_output_sync`),
  `is_initialized()`. — **Mechanism #2** front end; feeds steps 7–8, and the
  self-loopback re-injection in `handle_local_share_read`.
- **`ft/FtChannelManager.h`** — registry of FT input/output channels.
  `FtChannelManager(ident.string)`, `init(node_configs)`, `read_input(...)` (slot),
  `flush_inputs()`, `get_triple_string_output/get_output/get_dgram_input`. — Mechanism
  #2 plumbing + synced command conduit; cycle step 17.
- **`SlateSharer.h` family** —
  - *`SlateSharerSender`* `(Clock&)`; `init(name, source, shard, configs, config_list,
    channel, create_or_bind)`, `share()`. Local sender shares `shard_nonsync`; control
    senders share `shard_sync`. Cycle step 7.
  - *`SlateSharerReceiver`* `(Clock&, nano_t cycle_delay)`; `init(name, destination,
    shard, configs, config_list, in, create_or_bind)`. **Three instances →
    `slate_shared.{a,b,c}`** (the per-string stores that get voted).
  - *`SlateSharerManagerTripleString`* `(Clock&)`; `init(control_period, configs,
    process_name, identity, slate_control, node_configs, scaling&)`, `finalize(alert,
    channel_manager)`, `dispatch_pre_control()`/`dispatch_post_control()` (bracket the
    synced step). — **Mechanism #2/#3** sharing groups.
- **`SlateSyncer.h`** — checksum/recover the replicated `sync` shard (hotsync) over a
  dedicated FdBag. `SlateSyncer(Clock&, FdBag&, duplex_mode, control_period)`;
  `init_outputs(slate_local, role_inst, string, start_enabled=false)`,
  `init_inputs(shared_a,b,c, median, string)`, `init()`; per-cycle
  `dispatch_process` (step 3, apply hotsync), `dispatch_check` (step 9, desync detect),
  `dispatch_share` (step 15, hash + offer); `enable_transfer/enable_hotsync/
  set_single_peer_hotsync(bool)/disable_transfer/clear_counters`. — **Mechanism #4**;
  the commandable hotsync gates confirm SYSTEM_DESIGN's "policy is commandable".
- **`FtBootstrapper.h`** — supervisory sync state machine.
  `create(slate_local, slate_control_read_only, ident, control_period,
  time_sync_bootstrap_nodes={"satfc1"}, time_sync_mode=auto_time_sync,
  min_trigger_strings_mode, input_sync_mode, output_sync_mode, slate_syncer_mode,
  delays)`; static `populate_enums`; `dispatch()` (step 10), `is_synced()` (gates the
  synced step). Signals `time_sync_established_sig`, `full_sync_established_sig`. —
  sequences mechanisms #1–#4; the `satfc1` initiator matches SYSTEM_DESIGN.
- **`TimestampGatherer`/`TimestampSynchronizer.h`** — `TimestampGatherer(Clock&)`,
  `init(slate_local, slate_control_read_only)`, `update_local_timestamp()` (step 5),
  `update_clock_telemetry_timestamp()`. `TimestampSynchronizer::synchronized_timestamp_path`
  constant. — time-domain telemetry coherence.
- **`time_slave_preload.h`** — sim hooks `time_slave_control_begin/end()`,
  `time_slave_ds_dispatch_begin(is_synced)`. — SIL timing; bracket the synced step
  and the no-input-sync `fswsleep`.
- **`AdcScaler.h`** — raw→engineering scaling. `init(raw, scaled[, median], boards,
  bank_select[, exclude])`, `read_scaled()` (step 5), static `select_all_banks`. Per
  string: `adc_scaler_shared_{a,b,c}` scale each `slate_shared.{a,b,c}`. — input
  conditioning before the vote.
- **`FtraceTrap.h`** — kernel-trace snapshot on alarm. `create(clock, slate_control_ro,
  slate_local[, node_name])`, static `init_device(slate_control[, prefix])` per string,
  `dispatch()` (step 13). — diagnostics tied to cross-string alarms.

### L7 — Control + GNC + entry point
- **`ControlInterface.h`** — abstract control the runtime drives: `init_basic_identity`,
  `init_pre_slate_build(keychain, channel_manager, adc_boards, enum_registry,
  cycle_timers)`, `init_post_slate_build`, `get_scaling`, `start_cycle(control_time)`
  (step 2), `dispatch_synced() RUNTIME` (step 12). `BasicControl` implements it
  (present); `DroneControl`/`DroneGncControl` are leaves. — **Mechanism #3**
  deterministic identical computation.
- **`ControlState.h`** — *no construction site in scope; reconstruct from the control
  subtree.* Corresponds to the replicated `shard_sync` control state. **Open item.**
- **`ControlTask.h`** — abstract state-machine-managed unit; registered via
  `StateMachine::add_ctask(name, Handle<ControlTask>)`; observed subclasses
  `SlateAlarmTask`, `GncController`. Activated/deactivated by `StateMachine` (present)
  across states. — runs inside `dispatch_synced` (Mechanism #3).
- **`gnc/GncController.h`** — GNC task. `GncController("gnc_controller",
  satgnc_control_period)`, `init(slate_control, configs, factory, "mission_gnc_controller",
  "")`, `cmd_sig→StateMachine::handle_cmd`, added via `add_ctask("gnc", gnc)`. **(sat
  mission.)** — vehicle control law.
- **`gnc/GncComponentFactory.h`** — builds GNC components (`(clock, slate_control,
  state_registry, channel_manager, configs, cmd_table)`, `init`, `get_enum_registry`,
  `clear_config_cache`). **`gnc/StateRegistrySlateInterface.h`** — `StateRegistry`
  (`(gnc subslate)`, `init`, `lock`, `create_devices`, `init_alarms`, `save_snapshot`)
  + manager (`dispatch_inputs/outputs`). — GNC↔Slate coupling.
- **Entry point `main()`** — **does not exist.** Must construct `EventLoop`, the
  service/node directories, `NodeIdentity` from argv/config, the concrete
  `DroneFtRuntime`/`DroneGncFtRuntime`, register it with the loop, and run. — the
  missing top of the stack.

### L8 — Hardware (HAL) — **(sat)**
`FirmwareComm` (FPGA/ADC mmap DMA; DOs incl. watchdog GPIOs), `FirmwareCommAdcInit`
(`init_hardware`), `RealTimeClockInterface` (`/dev/rtc0`, `create`, `dispatch`,
`update_rtc_from_unix_time`), `WatchdogHeartbeat` (`/dev/watchdog`, `create`,
`dispatch`), `PpsManager`/`SwiftPpsInterface` (GNSS PPS), `PowerConverterInterface`
(+Distributed/Muxed; SAPC, `sweep_telem_size`), `TtcRuntime` (AD9361 RF over spidev).
— hardware I/O (FirmwareComm read/write = cycle step 4). **All satellite-board
specific.**

### L9 — Vehicle domain — **(sat → drone)**
`DroneGncControl` (the satgnc law), `DroneRegisterFlightComputerGnc`
(`register_satfc_gnc_components()`), `ColaBurnMonitor` (collision-avoidance burns),
`satellite_constants.h`/`satellite_rev_t`/`fcpu_board_rev_t`/`calibration_status_t`.
— **the spacecraft mission layer; replace wholesale for a drone (§6).**

---

## 4. Milestones (vertical slices, binary acceptance gates)

Each milestone is a thin slice that **builds and is verified** before the next. Gates
are binary (a grep returns empty / a test exits 0 / a number meets a bound), per the
`data-oriented-design` measurement protocol and the existing `scripts/check.sh`.

### M0 — Build spine + core primitives (L0)
- **Do:** author `core/{fsw,drone_types,fswtime,util}.h`, `hash/{Hash128,xxh}.h`,
  `static_vector.h`, `runtime.h` (annotation macros, possibly no-ops), real `B2`/`B2c`,
  the enum codegen + the present `*.enum.h` set; fix the **include-root layout** (the
  18 path-mismatch includes — decide one include root and move/rename files or set
  Bazel `strip_include_prefix`/`includes`); write a real `cc_library` BUILD for
  `bullwinkle` core + a unit-test target.
- **Gate:** `bazel build //vehicle/src/bullwinkle/...:core` green; `grep -rn '#include' |
  resolve` reports **0 unresolved internal includes within L0**; a `Hash128`/`B2c`
  round-trip + `static_vector` test passes.
- **Design check:** types are pointer-free and fixed-width (Slate Rule #4);
  `RUNTIME`/`INFRASTRUCTURE` compile. **DOD:** `objdump` confirms `static_vector`
  has no heap calls.

### M1 — Slate compiles, builds, hashes, votes (L1)
- **Do:** `SlateElement`, `SlatePathMap`, `SlateBuilderStore`, `slate_info`,
  `EnumRegistry`, `ReflectionManager`, `SlateDump`. The present `Slate*.{h,cc}`,
  `SlateCombiner`, `slate_tokens` now compile.
- **Gate:** a test builds a Slate, `create`s elements in each of the 7 shards,
  loads/stores by token, `roll_frame()` wipes cyclic, `compute_hash`/`compute_shard_deltas`
  agree across two identical Slates and differ on one mutation; a `SlateCombiner`
  test votes 3/3, 2/2, 1/1 by freshness and `downselect` avoids tearing.
- **Design check:** the shard table (7 shards, telemetered variants) and the
  permission boundary (`slate_permission_c_sync` only for control) match
  SYSTEM_DESIGN §Shards/§Permissions. **DOD:** no allocation in load/store hot paths.

### M2 — Event loop + loopback transport (L2)
- **Do:** `EventLoop`/`EventSource`/`EventList`/`Periodic`, `Clock`, `FdEventSink`,
  `io/Channel`/`DgramChannel`/`StreamChannel`/`DgramConnection`, forks, `Data*Channel`,
  `NullDgramChannel`, `BipBuffer`, `DataQueue`, net/sock/multicast utils, `Crc`,
  `Reconnector`, `ExponentialBackoffTimer`, `ServerFd`/`TcpServerFd`. The present
  `io/*`, `FdBag`, `Server`, `DeferredCallbackQueue` now compile.
- **Gate:** a `UdpConnection` loopback round-trip test (send N datagrams, receive N,
  bytes equal); a `DataDgramChannel` self-loopback test; `FdBag::select_once()`
  dispatches read/write events; reconnect fires on a forced close.
- **Design check:** datagram vs stream split, auto-reconnect, multicast, and the
  "even self traffic goes through datagrams via loopback" property (SYSTEM_DESIGN
  §Transport, §"Why this is not shared memory"). **DOD:** channel buffers are
  fixed-capacity `BipBuffer`/`DataQueue` (no per-datagram alloc); `perf stat` clean.

### M3 — Naming + config (L3)
- **Do:** `Configs`, `ServiceDirectory` (`service.h`, `vehicle_network.h`),
  `NodeIdentifier`, `ft/FtNodeConfig`; ship sample config data (a service directory, a
  node directory with the three strings' addresses, a `node_mgr` config).
- **Gate:** `service_directory().lookup("...", svc)` returns the configured
  host/port/proto; `verify_node_directory_segments(...)` passes for in-segment IPs and
  fails out-of-segment; `FtNodeConfigList::parse` + `get_node_output_buffer_size`
  return configured values.
- **Design check:** logical-name resolution + segment validation (SYSTEM_DESIGN
  §"Service directory / naming"). This unblocks the ring connection setup in M6.

### M4 — Telemetry end to end (L4)
- **Do:** `BwpFramer`/`BwpWriter`/`BwpChannelWriter`/`ByteQuotaFramer`,
  `TelemetryFlowInfo`/`TelemetryRelayFlow`/`TelemetryWriter`/`TelemetryConsumer`,
  `SlateTelemetryTask`, `Dgram/StreamChannelTelemetryTask`, `DeviceTelemetryFactory`,
  `GroundTelemetryRelay`. The present `TelemetryRelay.{h,cc}` now compiles.
- **Gate:** a `SlateTelemetryTask` frames a Slate group into BWP, writes to a
  `DataDgramChannel`, and a decoder recovers the values; the `ByteQuotaFramer`
  enforces a measured byte/sec bound (burst+recharge); a muxed group produces
  byte-identical frames given identical input (the cross-string check).
- **Design check:** BWP framing, flows/groups, byte-quota throttle, muxed-string
  identity, local-writer redirection (SYSTEM_DESIGN §"Telemetry out"). **DOD:** the
  framing path is bounded and allocation-free per cycle.

### M5 — Command path + security (L5)
- **Do:** `ExternalCommandDispatcher` + the five handlers, `CommandTable`,
  `external_command_util`, `CommonCommandFilter`, `ExternalCommandFilterCommon`,
  `Keychain`, `Ed25519Crypto`, the `.proto` files + codegen, an HSM **SIL stub**
  (software keys). The present `SlateCommandInterface`, `CommandQueueClient`,
  `CommandPump`, `CommandSender`, gRPC files now compile.
- **Gate:** set-by-hash and set-by-name apply to the Slate; `set_multi_by_hash` is
  atomic (inject one invalid set → all roll back); dedup drops a replayed seq num;
  the time filter rejects stale commands; an Ed25519-signed command verifies and a
  tampered one is rejected; **synced commands write `shard_sync`, nonsynced write
  `shard_nonsync`** (grep the create sites).
- **Design check:** the two command paths, name/hash addressing, 50-element atomic
  batch, Ed25519 two-keystore auth (SYSTEM_DESIGN §"Commands in"). **DOD:** the
  RUNTIME `dispatch()` path is allocation/exception-free.

### M6 — The TMR core, proven in SIL (L6) — *the keystone*
- **Do:** `FtSync`, `FtSimpleDataSharer`, `NodeIoManager`, `FtChannelManager`, the
  `SlateSharer` family, `SlateSyncer`, `FtBootstrapper`, `TimestampGatherer/Synchronizer`,
  `time_slave_preload`, `AdcScaler`, `FtraceTrap`. The present `FtRuntime`/`BasicControl`
  now compile.
- **Gate (single process):** a one-string self-loopback run reaches `is_synced()` and
  runs the cycle. **Gate (three processes, SIL):** launch `satfc1a/b/c` on loopback;
  prove (1) **time lock** — all three step on the same cadence (FtSync); (2) **input
  agreement** — after share+reshare+vote, the three control slates are byte-identical
  (`compute_hash` equal); (3) **majority survival** — kill one string, the other two
  keep running and stay agreed; (4) **hotsync recovery** — restart the killed string,
  it pulls the `sync` shard and rejoins (`SlateSyncer`), reaching agreement within N
  cycles; (5) **signed inputs** — a forged input datagram is rejected.
- **Design check:** this is the literal proof of the four agreement mechanisms +
  control cycle in SYSTEM_DESIGN §"Keeping the three units in agreement" / §"The
  control cycle". **DOD:** measure per-cycle WCET ≤ control period; the share/vote/
  hash paths are bounded and allocation-free.

### M7 — Full control cycle + entry point (L7)
- **Do:** finish `ControlState`/`ControlTask` (resolve from the control subtree),
  wire `GncController`/`GncComponentFactory`/`StateRegistry` (or a minimal control law
  for the slice), and write **`main()`** that builds the `EventLoop` + concrete runtime
  and runs. The present `DroneGncControl`/`DroneFtRuntime` compile against a HAL stub.
- **Gate:** a SIL run of the full `dispatch()` sequence (all 17 steps) across three
  strings produces a deterministic, identical `sync` shard each cycle, emits telemetry,
  and accepts a synced command that all three apply identically; a force-compact /
  restart resumes correctly.
- **Design check:** the complete control cycle and identical-computation property
  (SYSTEM_DESIGN §"The control cycle", §Mechanism #3).

### M8 — Hardware abstraction layer (L8)
- **Do:** define a **HAL interface** (sensors in, actuators/DO out, RTC, watchdog,
  time/PPS, power) with two implementations: a **SIL simulator** (already used by
  M2–M7) and **real drivers** for the target board. Keep `FirmwareComm` semantics
  (the only `mmap`, FPGA registers) behind the interface.
- **Gate:** the SIL sim and a bench HIL produce equivalent Slate inputs for a recorded
  scenario; watchdog pets within budget; RTC persists across restart.
- **Design check:** hardware I/O at cycle step 4; the FPGA mmap stays hardware-I/O,
  not IPC (SYSTEM_DESIGN §"Why this is not shared memory"). **DOD:** driver hot paths
  are zero-alloc, bounded, with measured latency.

### M9 — Drone domain pivot (L9) — Track B
- **Do:** replace the satellite mission layer with drone equivalents (§6): a drone
  `GncController` (attitude/position estimator + control law), motor-mixer + ESC/servo
  HAL, RC-link + failsafe, geofence/return-to-home, a drone time source (GPS/baro
  fusion, not satellite PPS), drone power monitoring (battery, not SAPC), and drop
  `ColaBurnMonitor`/TT&C/`satellite_*`. Replace `satellite_constants`/rev enums with
  `drone_*`.
- **Gate:** a SIL flight (sim dynamics) holds attitude/altitude, responds to RC,
  triggers failsafe on link loss, and respects geofence — all under the same
  triple-redundant runtime (M6 properties still hold).
- **Design check:** the framework (Slate/transport/FT/commands/telemetry) is
  vehicle-agnostic; only the control law + HAL change. SYSTEM_DESIGN's redundancy
  story is preserved.

### M10 — Production hardening
- **Do:** real-time guarantees (`SCHED_FIFO`, `mlockall`, CPU isolation/affinity,
  no-fault hot path), real keys/HSM, signed/verified boot, OTA, cross-compile
  toolchain for the flight board, HIL + soak + fault-injection test campaigns, and the
  `scripts/check.sh` gate green on the target.
- **Gate:** measured WCET < period with margin under load; injected single-unit faults
  never interrupt control (the SYSTEM_DESIGN promise) on real hardware; no allocation/
  page-fault in the hot path (verified). **DOD:** the measurement protocol is the
  acceptance evidence.

---

## 5. Verification & measurement strategy (DOD)

- **Per-component:** unit test (empty/full/error), and for hot code a microbench vs a
  scalar baseline + `objdump` audit that `RUNTIME` methods emit no `call malloc`/
  exception tables (*measurement_and_verification.md*).
- **SIL harness (M6–M9):** a launcher that runs `a/b/c` on loopback with a simulated
  clock (`time_slave_*`) and a fault injector (kill/restart a string, drop/forge a
  datagram, delay a link). The TMR property tests live here.
- **Determinism gate:** the `sync`-shard `compute_hash` must be equal across all
  fresh strings every cycle — the single strongest correctness signal for Mechanism #3.
- **Real-time gate:** per-cycle WCET measured (`perf`, cycle timers) against
  `FtSync::get_sync_period()`; the budget is a hard ceiling.
- **Coverage of failure modes:** a test per SYSTEM_DESIGN promise — single-unit loss,
  desync→hotsync recovery, forged-input rejection, command dedup/staleness, byte-quota
  throttle, muxed-frame identity.

---

## 6. Drone-vs-satellite replacement table (Track B)

| Satellite component (present/missing) | Drone replacement |
|---|---|
| `gnc/GncController` + `DroneRegisterFlightComputerGnc` (orbit/attitude, mission) | Drone GNC: attitude+position EKF, rate/attitude/position control, motor mixer |
| `common/ColaBurnMonitor` (conjunction/collision-avoidance burns) | Geofence / proximity / RTL-trigger monitor |
| `pps/PpsManager`, `pps/SwiftPpsInterface` (GNSS PPS) | Drone time/nav: GPS+baro+IMU fusion; PPS optional |
| `power/PowerConverterInterface` (+Distributed/Muxed, SAPC/solar) | Battery/ESC power monitor + current/voltage sensing |
| `flight-computer/TtcRuntime` (AD9361 RF TT&C) | RC link (e.g. CRSF/SBUS) + telemetry radio (MAVLink-style) |
| `flight-computer/FirmwareComm`/`FirmwareCommAdcInit` (FPGA DMA) | Drone sensor/actuator HAL: IMU/baro/mag/GPS in, ESC/servo PWM/DShout out |
| `fleet_client/command/CommandPayload`, `Ed25519Crypto` (NOC keys), fleet `.proto` | GCS command protocol + key hierarchy (keep Ed25519 auth model) |
| `satellite_constants/_rev/fcpu_board_rev/calibration_status` | `drone_*` constants/rev/board enums |
| **Reusable as-is:** Slate, transport (`io/*`), `FtSync`/`FtSimpleDataSharer`/`SlateSyncer`/`FtBootstrapper`/`FtChannelManager`, command engine, telemetry relay, `Keychain`, `hsm_type_t`, gRPC client templates | — keep |

The redundancy framework (L0–L6) is vehicle-agnostic; the pivot is L7–L9.

---

## 7. Risks & open questions

1. **Upstream framework source.** The ~172 missing files almost certainly exist in
   the originating monorepo. If obtainable, M0–M8 become *integration + include-path
   fixes*, not reimplementation. **Decide first:** port the real headers vs.
   re-implement to the inferred contracts. This is the single biggest schedule lever.
2. **`ControlState` / `ControlTask` internals** are not reconstructable from this
   slice (no construction sites) — they live in the control subtree (`DroneControl`/
   `StateMachine`). Needs the upstream source or a fresh minimal design.
3. **Generated artifacts** — the `*.enum.h` and `*.pb.h`/`*.grpc.pb.h` need their
   generators + source `.proto`/enum definitions, plus a Bazel codegen rule. None are
   present.
4. **Include-root inconsistency** — the present files live at paths that don't match
   their own `#include`s (18 path-mismatches; e.g. `drone/sat/util/...` vs
   `ground/drone/util/...`, top-level files included as `src/bullwinkle/all/...`).
   Resolve the include root *before* M0 or everything downstream re-breaks.
5. **Real-time platform** is unspecified (RTOS? PREEMPT_RT Linux? bare board?). The
   hard-real-time claims in SYSTEM_DESIGN need a concrete target before M10.
6. **HSM hardware** varies by board (CMRT/STSAFE/TrustZone present as variants); SIL
   uses a software-key stub, flight needs the real module.

---

*Contracts in §3/§6 were reverse-engineered from construction/dispatch sites in the
present code and cross-checked against `SYSTEM_DESIGN.md`; symbols with no in-tree
usage (`ControlState`, parts of `Clock`/`ServiceDirectory`/`TripleString`) are flagged
as design-only. Coding standards (§1) are the `data-oriented-design` skill applied to
this repo's existing Slate/`RUNTIME`/`Handle`/`Signal` idioms.*
