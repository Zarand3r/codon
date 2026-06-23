# Naming conventions — no SpaceX / Starlink allusions

This codebase descends from a SpaceX/Starlink flight-software slice. **All
SpaceX/Starlink-specific naming is being removed. Do not introduce identifiers
that allude to SpaceX, Starlink, or their house prefixes.** When you write or
touch code, follow the rules below.

## Applied — house prefixes → `fsw` ("flight software")

A mechanical, repo-wide rename already done (token-safe, case-sensitive):

| Old | New | Examples |
|-----|-----|----------|
| `sx` / `SX` / `Sx` | `fsw` / `FSW` / `Fsw` | `sxtime`→`fswtime`, `sxsleep`→`fswsleep`, `SX_DISALLOW_COPY_AND_ASSIGN`→`FSW_DISALLOW_COPY_AND_ASSIGN`, `SX_ASSERT`→`FSW_ASSERT`, `SxPerfGetFileName`→`FswPerfGetFileName`, `sx_recvfrom`→`fsw_recvfrom` |
| `Sac` / `sac` | `Fsw` / `fsw` | the assert / early-return family: `SacAbortIfNot`→`FswAbortIfNot`, `SacAssert`→`FswAssert`, `SacPrefix`→`FswPrefix`; header `core/sac.h`→`core/fsw.h` |
| `noc` | `operator` | NOC = network operations center: `noc_keystore`→`operator_keystore` |

**Deliberately preserved** (not the prefix): `absX` (a coordinate component) and
the shell keyword `esac`.

## To apply as the vehicle layer is reimplemented — Track B (drone pivot)

These are the **satellite vehicle domain**, not house prefixes. Substituting them
is part of the satellite→drone pivot (`ROADMAP.md` Track B; `IMPLEMENTATION_PLAN.md`
decisions **D15–D19**) — the right substitute depends on the drone design, so they
are **not** mechanical renames. The imported reference files keep these names until
the layer that owns them is reimplemented (it gets replaced wholesale). Targets:

| Old (alludes to Starlink / satellite) | Substitute |
|---|---|
| `Satellite` / `satellite` / `sat` | `Vehicle` / `vehicle` (neutral) — or the agreed drone term |
| `satfc` (satellite flight computer) | `fc` / `dronefc` |
| `satgnc`, `satgps` | `gnc`, `gps` (drop the `sat` prefix) |
| `Satellites::Fleet` (proto/API namespace) | a vehicle-neutral API namespace |
| `SAPC` (solar-array power converter) | drone power: battery / ESC monitor |
| `Ttc` / TT&C radio (`TtcRuntime`) | RC + telemetry link (`radio`) |
| `Cola` / `ColaBurnMonitor` (collision-avoidance burn) | geofence / hazard monitor |
| `noc` already handled above | — |

Do **not** rename these piecemeal in the imported reference tree — it adds churn
for files that will be replaced. Apply the substitute when you reimplement that
component, and if the substitute needs a design choice, raise it (R4).

## Rule going forward

1. **No new identifier may allude to SpaceX, Starlink, `sx`, `Sac`, or `noc`.**
2. Use **`fsw`** for flight-software house utilities and macros (asserts, time,
   sockets, perf, copy-suppression).
3. For vehicle-domain names prefer **neutral** (`vehicle`) or the agreed drone
   term; if the choice requires a design decision, surface it per
   `IMPLEMENTATION_PLAN.md` R4 rather than guessing.
