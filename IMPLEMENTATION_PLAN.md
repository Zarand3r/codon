# Codon Flight Software — Implementation Plan (TDD, gap-filling)

> **Status:** ready to execute. Derived from [`ROADMAP.md`](ROADMAP.md); every
> phase fills in *missing* pieces whose contracts were reverse-engineered from
> present usage and checked against [`SYSTEM_DESIGN.md`](SYSTEM_DESIGN.md). Coding
> style is the `data-oriented-design` skill (ROADMAP §1).
>
> This plan is **test-driven and strictly dependency-ordered**: a layer is proven
> correct before the layer above it is started. The Slate data model must pass its
> tests before the slate-sharing system is built; sharing + service + transport
> must pass before the fault-tolerant agreement layer; and so on.

---

## 0. Operating principles — NON-NEGOTIABLE, read before every step

These four rules dominate everything else. If a step would violate one, **stop**.

### R1 — Fill in what's missing. Do **NOT** extend the system design.
- Implement **only** what (a) appears as a usage site in the *present* code (a call,
  a member, a construction — cite `file:line`) **or** (b) is stated in
  `SYSTEM_DESIGN.md` (cite the sentence). Nothing else.
- **Every symbol you create must pass the Extension Check:** name the present
  usage site or the design sentence that requires it. If you cannot, you are
  extending the system — **STOP** and raise a Design Decision (R4). Do **not**
  invent API, add features, add config knobs, add abstraction layers, or
  "improve" the design. A bigger surface than the call sites demand is a defect.
- No speculative generality (DOD: *every abstraction is guilty until proven
  useful*). The inferred contract is the ceiling, not the floor.

### R2 — Test-Driven Development. Red → Green → Gate. No code before a failing test.
For each component/step:
1. **Red** — write the test(s) that encode the inferred contract **and** the
   `SYSTEM_DESIGN.md` property it must uphold. Run them; they MUST fail (compile or
   assert). A test that passes before implementation is wrong.
2. **Green** — write the **minimum** code to pass. Nothing the tests don't force
   (this is also how R1 is enforced mechanically — untested surface shouldn't exist).
3. **Refactor** — only with tests green; behavior-preserving; re-run.
4. A bug fix gets a test that reproduces the **category**, not just the instance,
   before the fix (catches siblings).
Never weaken, skip, comment out, or delete a test to make a gate pass.

### R3 — Gate before proceeding. A phase is closed only when its gate is green.
- Each phase has a **binary acceptance gate** (a grep returns empty / a test exits
  0 / a measured number meets a bound). `scripts/check.sh` must be green.
- **You may not start phase N+1 while phase N's gate is red.** This is the
  dependency discipline the user asked for: prove the Slate before sharing, prove
  sharing+service+transport before fault tolerance, etc.
- A phase also closes only when its **Design-Consistency Check** passes (the
  `SYSTEM_DESIGN.md` claim it implements is demonstrated by a test) and its
  **Extension Check** passes (R1).
- **Docs move with the code.** A component/phase is not done until its entry in
  [`docs/implementation.md`](docs/implementation.md) is added/updated (contract,
  test location, verification status, any deferred flags). Stale docs are a defect,
  not a follow-up.

### R4 — Ambiguity ⇒ surface a Design Decision to a human. Do not guess.
- When a missing piece's contract **cannot be inferred** from present usage or the
  design — too many unknowns, a schema that doesn't exist, an internal not exercised
  by any call site, a platform choice — **STOP**. Append an entry to **§8 Design
  Decisions Needed** (Statement / Options / Recommendation / Blocks-phase) and do
  not proceed past the phase it blocks until a human resolves it.
- Prefer stopping over inventing. A documented question beats a fabricated API that
  silently diverges from the real system. (This is the elves/PPE "surface tensions,
  don't silently resolve them" rule made mandatory here.)
- The known blockers are pre-listed in §8; resolve the **Phase-0 blockers first.**

> **One-line creed:** *Build only what the call sites and the design demand, prove
> each layer with a failing-then-passing test before building on it, and ask a human
> the moment a gap is ambiguous.*

---

## 1. Steps at a glance (phases + dependency graph)

Phases mirror `ROADMAP.md` milestones M0–M10. Critical path is strictly linear
through P6 (each proves the layer the next relies on); P8–P9 (drone, hardening)
fork after P7.

```
P0  Decisions + build spine + core primitives        (M0)   ← resolve §8 blockers FIRST
P1  Slate data model                                  (M1)   depends P0
P2  Event loop + transport (loopback)                 (M2)   depends P1
P3  Naming + config (service/node directory)          (M3)   depends P2
P4  Slate sharing + voting (no time-sync yet)         (part of M6) depends P1,P2,P3
P5  Telemetry out                                     (M4)   depends P2,P3
P6  Commands in + security                            (M5)   depends P1,P2,P3
P7  Fault-tolerant agreement (time/sync/recover/boot) (M6)   depends P4,P5,P6
P8  Control integration + entry point                 (M7)   depends P7
   ── critical path ends; below forks ──
P9  Hardware abstraction layer                        (M8)   depends P8
P10 Drone domain pivot                                (M9)   depends P8,P9   [Track B]
P11 Production hardening                              (M10)  depends all
```

Critical path: **P0 → P1 → P2 → P3 → P4 → P6 → P7 → P8**. P5 parallels P6; P9–P11
follow P8. (Note: P4 "sharing" is split out of ROADMAP M6 and proven *before* the
full FT layer P7, per the user's ordering: slate → sharing → service → fault
tolerance.)

---

## 2. Properties to preserve (each tied to the phase+test that proves it)

These are the `SYSTEM_DESIGN.md` invariants as **testable gates**. A phase that
demonstrably weakens a property cannot close.

| # | Property (from SYSTEM_DESIGN) | Proven by |
|---|---|---|
| **P1** | Slate stores **no pointers** — bytes are position-independent (valid on another machine/run) | P1: serialize a built Slate, relocate the buffer, re-read every element equal; grep finds no pointer-typed slate elements |
| **P2** | Layout **frozen** after build — structure cannot change post-`finalize` | P1: `create()` after `finalize()` is rejected; layout hash stable across reopen |
| **P3** | **Permission boundary** — only control creates `sync`; control barred from `nonsync` | P1: runtime slate `create` in `sync` rejected; control slate `create` in `nonsync` rejected |
| **P4** | **Byte-for-byte agreement** — identical Slates hash-equal; deltas pinpoint a single mutation | P1: two identical Slates → equal `compute_hash`; mutate one element → `compute_shard_deltas` returns exactly that region |
| **P5** | **No shared memory** — no `shm`/`MAP_SHARED`/shared pages; only mmap is the FPGA HAL | P2 & a standing grep test: `MAP_SHARED`/`shm_open` absent except the hardware-register mmap |
| **P6** | **One inter-unit path** — all cross-unit data is a serialized datagram, even self/loopback | P4: self-share round-trips through a `DataDgramChannel` loopback, not a memory copy |
| **P7** | **Time alignment** — all strings share one cadence; timing step runs first and sets cycle time | P7: 3 SIL strings step on one cadence; `FtSync.dispatch` precedes all other components |
| **P8** | **Input agreement** — after share+reshare+vote, all strings hold an identical input set | P4/P7: the three control slates are `compute_hash`-equal after the vote |
| **P9** | **Median voting with freshness** — 3/3→2/2→1/1 by freshness; stale source dropped; downselect avoids tearing | P4: combiner unit tests for each fall-back and the downselect path |
| **P10** | **Signed inputs** — a forged input datagram is rejected | P4/P6: Keychain-verify test rejects a tampered datagram |
| **P11** | **Deterministic identical computation** — identical voted inputs ⇒ identical `sync` shard each cycle | P7/P8: cross-string `sync`-shard hash equal every cycle |
| **P12** | **Single-unit-loss tolerance** — losing any one string never interrupts control | P7: kill one of three; other two continue and stay agreed |
| **P13** | **Hotsync recovery** — a rebooted/desynced string pulls the whole `sync` shard and rejoins | P7: restart a string; it reaches agreement within N cycles, no cold start |
| **P14** | **Two command paths** — replicated→`sync`, private→`nonsync`; multi-set atomic; dedup/time-filter | P6: command tests for shard targeting, atomic rollback, dedup, staleness |
| **P15** | **Rate-limited, framed telemetry** — byte-quota bound holds; muxed groups frame identically | P5/P7: measured byte/sec ≤ bound; muxed frame bytes equal across strings |
| **P16** | **No-extension** — every implemented symbol traces to a present usage site or a design sentence | every phase: the Extension Check audit |
| **P17** | **Hot-path discipline (DOD)** — `RUNTIME` methods allocate/throw/block nothing | every phase with `RUNTIME` code: `objdump` audit + WCET |

> Each property is a **merge gate, not an aspiration**. If a step can't satisfy its
> property with a test, the contract was mis-inferred → raise a Design Decision (R4).

---

## 3. Phase format

Every phase below uses the same blocks:
- **Goal / depends on** — one line + prerequisites (gate of those must be green).
- **Design claims satisfied** — the `SYSTEM_DESIGN.md` sentences this phase implements.
- **Tests first (Red)** — the exact tests to write before any code.
- **Implement (Green)** — the missing files to fill, minimum to pass. (Contracts in
  `ROADMAP.md` §3.)
- **Acceptance gate** — binary close condition.
- **Extension check** — R1 audit specific to this phase.
- **Design decisions** — R4 escalations that block this phase (if any).

---

## P0 — Decisions, build spine, core primitives  (ROADMAP M0)

**Goal / depends on:** resolve the Phase-0 design blockers (§8 D1–D4), stand up the
Bazel `cc_library`/test targets, fix the include-root layout, implement L0
primitives. Depends on nothing — but **§8 D1–D4 must be answered first** (R4).

**Design claims satisfied:** pointer-free fixed-width values (Slate Rule #4 substrate);
the build/test harness for everything after.

**Tests first (Red):**
- `Hash128`/`xxh` round-trip + known-vector test.
- `B2`/`B2c` span bounds + `static_vector<T,N>` full/empty/overflow test.
- A build smoke test: the L0 `cc_library` compiles; **an include-resolution test**
  (a script) asserts 0 unresolved internal includes *within L0*.

**Implement (Green):** `core/{fsw,drone_types,fswtime,util}.h`, `hash/{Hash128,xxh}.h`,
`static_vector.h`, `runtime.h` (annotation macros), real `B2`/`B2c`, `enum/auto_enum.h`
+ the codegen for `*.enum.h`, BUILD targets. Fix the 18 path-mismatch include roots.

**Acceptance gate:** L0 library builds; include-resolution test → 0 unresolved at L0;
primitive tests green.

**Extension check:** every macro/type traces to a present usage site (e.g.
`FswAbortIfNot` in `Slate.cc:65`, `UINT8` in `SlateCombiner.cc:281`). The enum members
come from present usage (`num_slate_shard_t==7`, etc.) — do not add enum values no
call site uses.

**Design decisions (BLOCKERS — resolve before starting):** D1 (port upstream vs
reimplement), D2 (include-root choice), D3 (enum/proto codegen toolchain), D4 (RT
target platform — needed for later WCET gates, decide direction now).

---

## P1 — Slate data model  (ROADMAP M1)  ·  *prove this before anything builds on it*

**Goal / depends on:** the Slate builds, stores/loads by token, freezes, rolls the
cyclic shard, hashes, diffs, votes. Depends on P0.

**Design claims satisfied:** "A single named model of all state"; build-phase-then-
frozen; no pointers; hash/byte-compare/deltas; the 7-shard memory policy; the
creation-time permission boundary.

**Tests first (Red):**
- **P1/P2:** build a Slate, `create` elements in all 7 shards, load/store by token,
  serialize→relocate→re-read equal; `create()` after `finalize()` rejected.
- **P3:** runtime-permission slate `create` in `sync` → rejected; control-permission
  slate `create` in `nonsync` → rejected (exact `slate_permission_deny` semantics).
- **P4:** two identical Slates → equal `get_shard_layout_hash` + `compute_hash`;
  mutate one element → `compute_shard_deltas` returns exactly that offset/region;
  `swap_shard_buffer` installs a peer buffer and re-equalizes the hash.
- **roll_frame:** cyclic shard reverts to template; references invalidated; leaked
  token warning fires.
- **SlateCombiner (P9):** 3/3, 2/2, 1/1 median by freshness; stale source
  (`fresh_age_tok` > `stale_threshold`) dropped; `downselect` takes one source whole
  (no tearing).

**Implement (Green):** `SlateElement`, `SlatePathMap`, `SlateBuilderStore`,
`slate_info`, `EnumRegistry`, `ReflectionManager`, `SlateDump`. (Contracts: ROADMAP
§3 L1.) The present `Slate*`, `SlateCombiner`, `slate_tokens` now compile.

**Acceptance gate:** all above tests green; the 7-shard table and permission
behavior match `SYSTEM_DESIGN.md` §Shards exactly.

**Extension check:** shards = exactly the 7 in `num_slate_shard_t`; permission values
= exactly those in `init_pre_control_early`. No extra shard, policy, or accessor.

**Design decisions:** none expected (Slate is fully exercised by present code). If a
`slate_info<T>` trait for a type with no present usage is "needed," it's an extension
→ R4.

---

## P2 — Event loop + transport (loopback)  (ROADMAP M2)

**Goal / depends on:** event loop dispatches fd events; datagram/stream channels do
loopback round-trips; auto-reconnect works. Depends on P1.

**Design claims satisfied:** "Why this is not shared memory" (one datagram path,
loopback for self/collocated); "Transport and naming" channel layer (buffered duplex,
readiness signals, transparent re-establish, multicast/broadcast).

**Tests first (Red):**
- **P6:** a `UdpConnection` loopback sends N datagrams → receives N, bytes equal; a
  `DataDgramChannel` self-loopback re-injects a datagram (no memory aliasing).
- `FdBag::select_once()` dispatches read/write; forced socket close → reconnect fires
  (`ExponentialBackoffTimer`).
- **P5 (standing test):** grep the whole tree → `MAP_SHARED`/`shm_open` appear **only**
  in the FPGA-register HAL path, never for Slate/IPC.
- **P17:** the channel write/read hot path emits no allocation (`objdump` audit).

**Implement (Green):** `EventLoop`/`EventSource`/`EventList`/`Periodic`, `Clock`,
`FdEventSink`, `io/{Channel,DgramChannel,StreamChannel,DgramConnection,*Fork,Data*Channel,
NullDgramChannel,AnyDgramConnection,TcpServerFd}`, `BipBuffer`, `DataQueue`, net/sock/
multicast utils, `Crc`, `Reconnector`, `ExponentialBackoffTimer`, `ServerFd`. (ROADMAP §3 L2.)

**Acceptance gate:** loopback + reconnect tests green; the no-shared-memory grep test
green; channel buffers are fixed-capacity (no per-datagram alloc).

**Extension check:** channel API surface = exactly the methods/signals present code
calls (the signal shapes are pinned in ROADMAP §3 L2). No new transport modes.

**Design decisions:** none expected.

---

## P3 — Naming + config  (ROADMAP M3)

**Goal / depends on:** logical service names resolve to host/port/proto; node
directory validates segments; node config parses. Depends on P2.

**Design claims satisfied:** "Transport and naming" — endpoints never hardcoded,
service/node directories, segment validation, topology rearrangeable.

**Tests first (Red):** `service_directory().lookup` returns configured endpoint;
`verify_node_directory_segments` accepts in-segment, rejects out-of-segment;
`FtNodeConfigList::parse` + `get_node_output_buffer_size` return configured values.
Ship a sample service directory + 3-string node directory + `node_mgr` config.

**Implement (Green):** `Configs`, `ServiceDirectory` (`service.h`, `vehicle_network.h`),
`NodeIdentifier`, `ft/FtNodeConfig`. (ROADMAP §3 L3.)

**Acceptance gate:** resolution + validation tests green.

**Extension check:** segment enum values = exactly those used
(`vehicle_network_segment_*` as cited). Don't add network segments.

**Design decisions:** **D5** — the concrete network topology / addressing for the
three strings (sample data is fine for SIL; real addressing is a human input).

---

## P4 — Slate sharing + voting  (part of ROADMAP M6)  ·  *prove sharing before full FT*

**Goal / depends on:** one unit can serialize a configured value-set, send it over a
channel, a receiver deposits it into a per-string slate, and the combiner votes —
**without** time-sync yet (single process, loopback). Depends on P1, P2, P3.

**Design claims satisfied:** "Communication style" (configuration-driven static
value-path lists, batched messages); "Input agreement and voting" minus the time/ring
choreography (that is P7); signed inputs.

**Tests first (Red):**
- **P8/P6:** `SlateSharerSender` packs a `sharer_config_v` set into one datagram;
  `SlateSharerReceiver` deposits it into `slate_shared.{a,b,c}`; round-trip values
  equal. Batch test: the whole configured set rides one datagram (not N).
- **P9:** combiner votes the three received slates → control slate; 3/3·2/2·1/1 +
  downselect (reuse P1 combiner tests at integration scale).
- **P10:** a datagram with a bad `Keychain` signature is rejected; a valid one accepted.
- Self-share path (P6): a unit's own inputs go out and come back via loopback and land
  identically (the "even feeding its own data back goes through messaging" claim).

**Implement (Green):** `SlateSharer{Sender,Receiver,Manager}`, `Keychain`,
`NodeIoManager` (enough to emit `data_sig`), `ft/FtChannelManager` (enough for
`read_input`/channel vending). (ROADMAP §3 L5/L6.) `FtSimpleDataSharer` *exchange* may
be stubbed to a single-process loopback here; the **ring/two-pass** choreography is P7.

**Acceptance gate:** share→receive→vote round-trip green; signature reject green; batch
(one datagram per set) verified.

**Extension check:** the sharer config is a *static list from config* — do not add a
dynamic subscribe API (the design explicitly forbids a topic broker).

**Design decisions:** **D6** — `Keychain` key provisioning for SIL (software keys);
real keys are P11/human. **D7** — `sharer_config_v` default contents
(`sharer_get_default_local_config_list`) if not fully determinable from config.

---

## P5 — Telemetry out  (ROADMAP M4)  ·  parallels P6

**Goal / depends on:** Slate framed into BWP, rate-limited, sent to a resolved
destination or redirected to a local consumer. Depends on P2, P3.

**Design claims satisfied:** "Telemetry out" — framed, byte-quota rate-limited,
resolved destinations, local-consumer redirection; muxed-group identical framing.

**Tests first (Red):** **P15:** `SlateTelemetryTask` frames a group → `DataDgramChannel`
→ decoder recovers values; `ByteQuotaFramer` enforces a *measured* byte/sec bound;
a muxed group frames byte-identically for identical input; `redirect_service` sends to
a local writer, not the socket.

**Implement (Green):** `BwpFramer`/`BwpWriter`/`BwpChannelWriter`/`ByteQuotaFramer`,
`TelemetryFlowInfo`/`TelemetryRelayFlow`/`TelemetryWriter`/`TelemetryConsumer`,
`SlateTelemetryTask`, `Dgram/StreamChannelTelemetryTask`, `DeviceTelemetryFactory`,
`GroundTelemetryRelay`. (ROADMAP §3 L4.) `TelemetryRelay.{h,cc}` now compiles.

**Acceptance gate:** frame-decode, measured rate bound, muxed-identity, redirect tests
green.

**Extension check:** flow/group fields = exactly `TelemetryFlowInfo`'s parsed fields.
No new telemetry transport.

**Design decisions:** none expected (relay is fully exercised by present code).

---

## P6 — Commands in + security  (ROADMAP M5)  ·  parallels P5

**Goal / depends on:** inbound command deframed → time-filtered → deduped →
authenticated → applied by name/hash, on the correct shard. Depends on P1, P2, P3.

**Design claims satisfied:** "Commands and telemetry — Commands in"; two paths
(replicated→`sync`, private→`nonsync`); atomic multi-set; Ed25519 auth.

**Tests first (Red):** **P14:** set-by-hash + set-by-name apply to the Slate;
`set_multi_by_hash` atomic (inject one invalid → all roll back); dedup drops a replayed
seq; time-filter rejects stale; **synced dispatcher writes `sync`, nonsynced writes
`nonsync`** (assert shard of the created elements). **P10:** Ed25519-signed command
verifies; tampered rejected (two-keystore unwrap).

**Implement (Green):** `ExternalCommandDispatcher` + the five handlers, `CommandTable`,
`external_command_util`, `CommonCommandFilter`, `ExternalCommandFilterCommon`,
`Ed25519Crypto`, the `.proto` + codegen, HSM **SIL stub**. (ROADMAP §3 L5.) Present
`SlateCommandInterface`/`CommandQueueClient`/`CommandPump`/`CommandSender`/gRPC compile.

**Acceptance gate:** all P14/P10 command tests green; RUNTIME `dispatch()` allocation-free.

**Extension check:** handlers = exactly the five the present chain pushes; the 50-element
multi-command cap is from `multi_command_v` — don't change it.

**Design decisions:** **D8** — the command `.proto` schemas
(`command_queue/service`, `security/{signed_data,tbs_command}`) are **not in the tree**
and not fully inferable; the field accessors used are known (ROADMAP §3 L5) but the
full schema is a **human input / upstream artifact**. **D9** — HSM stub acceptable for
SIL? (real HSM is P11.)

---

## P7 — Fault-tolerant agreement  (ROADMAP M6)  ·  *the keystone; prove TMR end-to-end*

**Goal / depends on:** three independent processes behave as one — time-locked,
input-agreed, identical-compute, recoverable. Depends on **P4, P5, P6** all green.

**Design claims satisfied:** all of "Keeping the three units in agreement" (the four
mechanisms) + "The control cycle".

**Tests first (Red):** build the **SIL harness** (3 processes `a/b/c` on loopback, sim
clock, fault injector). Then:
- **P7 (time):** all three step on one cadence; `FtSync.dispatch` runs first and sets
  control time; `satfc1` is the initiator.
- **P8/P11 (agree+determinism):** after share+reshare(ring, two-pass)+vote, the three
  control slates are `compute_hash`-equal **every cycle**.
- **P12 (loss):** kill one string → the other two continue and stay agreed.
- **P13 (recovery):** restart the killed string → it pulls the `sync` shard wholesale
  (`SlateSyncer`) and reaches agreement within N cycles; the commandable
  `enable_hotsync`/`force_disable_transfer` gates behave.
- **P10 (forged):** a forged input datagram is rejected mid-ring.
- **P17 (RT):** measured per-cycle WCET ≤ `FtSync::get_sync_period()`.

**Implement (Green):** `FtSync`, the **ring/two-pass** `FtSimpleDataSharer`,
`SlateSyncer`, `FtBootstrapper`, `TimestampGatherer`/`TimestampSynchronizer`,
`time_slave_preload`, `AdcScaler`, `FtraceTrap`; complete `NodeIoManager`/
`FtChannelManager`. (ROADMAP §3 L6.) Present `FtRuntime`/`BasicControl` compile.

**Acceptance gate:** the full SIL TMR suite (P7,P8,P10,P11,P12,P13) green; WCET bound met.
**This gate is the proof the system achieves the SYSTEM_DESIGN redundancy model.**

**Extension check:** the ring is exactly left/right × share/reshare (4 conns); node
types = exactly the 5 in `ft_node_type_t`; no new sync mode.

**Design decisions:** **D10** — `slate_syncer_duplex_mode`/bootstrapper delay constants
if not config-derived. **D11** — the SIL fault-injection scenarios to certify against
(human picks the acceptance set).

---

## P8 — Control integration + entry point  (ROADMAP M7)

**Goal / depends on:** the full `dispatch()` runs across three strings with a real (or
minimal) control law and a `main()` that wires `EventLoop` + runtime. Depends on P7.

**Design claims satisfied:** "Identical computation"; "The control cycle" (all steps);
end-to-end command+telemetry under the redundant runtime.

**Tests first (Red):** SIL run of all 17 cycle steps × 3 strings → deterministic
identical `sync` shard each cycle; a synced command is applied identically on all
three; a forced compaction/restart resumes correctly; telemetry emitted.

**Implement (Green):** resolve `ControlState`/`ControlTask` (**see D12**), wire
`GncController`/`GncComponentFactory`/`StateRegistry` (or a minimal control law for the
slice), write **`main()`**. (ROADMAP §3 L7.)

**Acceptance gate:** full-cycle SIL determinism + command/telemetry e2e green.

**Extension check:** `dispatch()` step order = exactly `FtRuntime`'s; do not add steps.

**Design decisions:** **D12 (HARD)** — `ControlState`/`ControlTask` internals are **not
reconstructable** from the present slice (no construction sites). Either obtain the
upstream control subtree or get a human to specify the minimal control-task contract.
**Blocks P8.** **D13** — what minimal control law to run in SIL if the real GNC isn't
ported.

---

## P9 — Hardware abstraction layer  (ROADMAP M8)  ·  Track A hardware

**Goal / depends on:** a HAL interface with a SIL simulator (used by P2–P8) and real
drivers for a target board. Depends on P8.

**Design claims satisfied:** hardware I/O at cycle step 4; the FPGA mmap stays
hardware-I/O, not IPC (preserves P5).

**Tests first (Red):** SIL sim and a bench HIL produce equivalent Slate inputs for a
recorded scenario; watchdog pets within budget; RTC persists across restart.

**Implement (Green):** define the HAL interface; SIL sim + real drivers for
`FirmwareComm`/`FirmwareCommAdcInit`/`RealTimeClockInterface`/`WatchdogHeartbeat`/
`PpsManager`/`SwiftPpsInterface`/`PowerConverterInterface`/`TtcRuntime`. (ROADMAP §3 L8.)

**Acceptance gate:** sim↔HIL equivalence + watchdog + RTC tests green; driver hot paths
zero-alloc/bounded (measured).

**Design decisions:** **D14** — the target board + its device map (FPGA registers, spidev,
GPIO) is a **human/hardware input**. **Blocks real drivers** (SIL sim can proceed).

---

## P10 — Drone domain pivot  (ROADMAP M9)  ·  Track B  ·  **leaves the framework intact**

**Goal / depends on:** replace the *satellite vehicle layer* with a drone layer, under
the **same** redundant runtime (P7 properties still hold). Depends on P8 (and P9 HAL).

> R1 caveat: this is the **one** place the system is intentionally changed — and only
> the *vehicle layer* (control law + HAL + domain monitors), never the redundancy
> framework. The drone components are still TDD'd against the same P1–P17 properties.
> All drone-domain contracts (which have no present usage) are **Design Decisions**,
> because there is no existing code to infer them from.

**Tests first (Red):** SIL flight (sim dynamics) holds attitude/altitude, responds to
RC, triggers failsafe on link loss, respects geofence — all while P11/P12/P13 (identical
compute, single-unit loss, recovery) still pass.

**Implement (Green):** per ROADMAP §6 — drone GNC (estimator + control law), motor
mixer + ESC/servo HAL, RC link + failsafe, geofence/RTH, drone time/power; drop
`ColaBurnMonitor`/TT&C/`satellite_*`; add `drone_*` constants/enums.

**Acceptance gate:** SIL drone flight + retained TMR properties green.

**Design decisions (ALL of L9 — these are designs, not gap-fills):** **D15** drone
control law & estimator; **D16** airframe/mixer/ESC protocol; **D17** RC link + failsafe
policy; **D18** geofence/RTH behavior; **D19** drone time/power sources. Each is a human
design decision (Track B is explicitly *beyond* filling the present gaps).

---

## P11 — Production hardening  (ROADMAP M10)

**Goal / depends on:** real-time guarantees, real keys/HSM, signed boot, OTA, HIL/soak/
fault-injection on hardware. Depends on all.

**Tests first (Red):** WCET < period with margin under load on target; injected
single-unit faults never interrupt control on real hardware; no alloc/page-fault in the
hot path (verified); signed-boot + OTA acceptance.

**Implement (Green):** `SCHED_FIFO`/`mlockall`/CPU isolation, real HSM, signed/verified
boot, OTA, cross-compile toolchain, test campaigns.

**Acceptance gate:** the hardware acceptance suite green; `scripts/check.sh` green on target.

**Design decisions:** **D20** RT platform specifics (from D4); **D21** key management &
secure-boot chain; **D22** OTA mechanism. Human/program inputs.

---

## 4. Golden-path integration test (the spine — grows each phase)

One fixed scenario, run after **every** phase from P4 on; red ⇒ the most recent phase
caused it.

- **From P1:** a built Slate + recorded element set → stamped layout/content hash.
- **From P4:** share the recorded inputs through loopback → voted control slate matches
  a stamped golden slate hash.
- **From P7:** 3-string SIL run of a recorded input trace → each string's per-cycle
  `sync`-shard hash matches the stamped golden sequence (this is the determinism +
  agreement spine); kill+restart injected at a fixed cycle → recovery by cycle K.
- **From P8:** full cycle incl. a scripted command + telemetry capture → golden artifact.

Update the golden artifact **only** at the end of a phase that intentionally changes it;
a mid-phase golden change is a red flag.

---

## 5. Iteration loop (when a test fails)

1. Read the failing assertion verbatim. A failing test is information.
2. Decide: is the **test** wrong (mis-inferred contract → possibly an R4 Design
   Decision) or the **code** wrong? Fix the right one.
3. If the code: minimum change OR a clean from-scratch rewrite of that file against the
   test — whichever is smaller. No scope creep to escape a stuck step.
4. Re-run the failing test, then the golden-path spine. Green ⇒ done.
5. **Stuck > 30 min or the contract is genuinely ambiguous → STOP, write a Design
   Decision (R4), do not invent.**

---

## 6. "Achieves the system design" — verification matrix

Every `SYSTEM_DESIGN.md` section maps to a property + the phase/test that proves it. The
plan *achieves the design* iff every row's gate is green.

| SYSTEM_DESIGN section | Property | Phase |
|---|---|---|
| Purpose & redundancy (end-to-end: inputs/state/outputs) | P8, P11/P12/P13, output-vote | P4, P7 |
| A single named model of all state | P1, P2 | P1 |
| Build phase, then frozen / no pointers / hash-compare | P1, P2, P4 | P1 |
| Shards — memory policy + permission boundary | P3 | P1 |
| Why this is not shared memory | P5, P6 | P2 (+ standing grep) |
| Transport and naming | P6 (loopback), naming | P2, P3 |
| Communication style & granularity (config-driven, batched) | P8 (batch), no-broker | P4 |
| 1. Time alignment | P7 | P7 |
| 2. Input agreement and voting | P8, P9, P10 | P4 (vote) + P7 (ring/time) |
| 3. Identical computation, optional output voting | P11, output-vote | P7, P8 |
| 4. State recovery | P13 | P7 |
| The control cycle | P11 (17-step determinism) | P8 |
| Commands and telemetry | P14, P15 | P6, P5 |

If a row has no green gate, the design is **not** yet achieved — that is the definition
of done for this plan.

---

## 7. Out of scope (do not build — R1 boundary)

- Anything not traceable to a present usage site or a `SYSTEM_DESIGN.md` sentence
  (R1/P16). New features, knobs, abstractions, transports, sync modes, shards.
- Extending the design "while we're here." The contract is the ceiling.
- Track B (drone domain, P10) is **not** gap-filling — it is new design behind explicit
  Design Decisions (D15–D19); do not start it as if it were inferable.
- Performance work beyond the stated WCET/rate gates (DOD: optimize the measured
  bottleneck only).

---

## 8. Design Decisions Needed (the escalation queue — R4)

Resolve before the phase each blocks. **D1–D4 block P0 and must be answered first.**

> **Resolved 2026-06-18.** **D1 = reimplement to inferred contracts** (full TDD
> build — this plan as written). **D4 = Linux PREEMPT_RT on the flight board**
> (WCET / `SCHED_FIFO` / `mlockall` gates live at P11; P1–P8 run SIL on a dev
> host). **D2 (default) = single include root `vehicle/`** so `#include "src/…"`
> resolves (Bazel `strip_include_prefix`/`includes`); each reimplemented header is
> placed to match its include path. **D3 (default) = Bazel `genrule`** for
> `*.enum.h` codegen (and `*.pb.h` at P6). D5+ are resolved at the phase they block.

| ID | Decision | Options / note | Blocks |
|----|----------|----------------|--------|
| **D1** | **Port the upstream framework headers, or reimplement to the inferred contracts?** | Porting turns most phases into integration; reimplementing is the inferred-contract path. *Biggest lever.* | P0 (all) |
| **D2** | Include-root layout (resolve the 18 path-mismatches) | one root + Bazel `strip_include_prefix`, vs move files | P0 |
| **D3** | Codegen toolchain for `*.enum.h` and `*.pb.h` | which generator + Bazel rule | P0, P6 |
| **D4** | Real-time target platform | RTOS / PREEMPT_RT Linux / bare board — sets WCET gates | P0 direction, P11 |
| **D5** | Network topology/addressing for the 3 strings | sample for SIL; real is human input | P3 |
| **D6** | `Keychain` key provisioning (SIL) | software keys for SIL; real keys P11 | P4 |
| **D7** | Default `sharer_config_v` contents | if not fully config-derivable | P4 |
| **D8** | Command `.proto` schemas (not in tree) | upstream artifact / human-supplied | P6 |
| **D9** | HSM software stub acceptable for SIL? | yes for SIL; real HSM P11 | P6 |
| **D10** | `slate_syncer_duplex_mode` + bootstrapper delays | config vs human | P7 |
| **D11** | SIL fault-injection certification set | human picks acceptance scenarios | P7 |
| **D12** | **`ControlState` / `ControlTask` internals — not inferable** | obtain upstream control subtree, or human specifies minimal contract | **P8 (hard)** |
| **D13** | Minimal SIL control law if real GNC not ported | human | P8 |
| **D14** | Target board + device map | hardware input | P9 (real drivers) |
| **D15–D19** | Drone domain: control law, mixer/ESC, RC+failsafe, geofence/RTH, time/power | **all human design (Track B)** | P10 |
| **D20–D22** | RT platform specifics, key mgmt/secure boot, OTA | program inputs | P11 |
| **D23** | `digest_xxh128` is MurmurHash3-x64-128, not XXH3-128 | fine now (internal, deterministic); **benchmark shard-hash throughput vs. control period at P7** and vendor XXH3 behind the same signature if it's a per-cycle bottleneck | P7 |
| **D24** | `FswAssert` is always-on `std::abort()` | reconcile prod semantics with the failure policy — flight wants **fail-to-safe-state, not `abort()`**, and the perf-assert family is compiled out in prod | P1 |

> **Protocol:** when execution reaches a phase whose blocker is unresolved, **STOP**,
> present the decision (Statement / Options / Recommendation), and wait. Never fabricate
> a contract to keep moving (R4).

---

*This plan fills only what is missing (R1), proves each layer with a failing-then-passing
test before building the next (R2/R3), surfaces every ambiguous gap as a human Design
Decision (R4), and is "done" only when every row of the §6 matrix is green — which is the
operational definition of "achieves `SYSTEM_DESIGN.md`."*
