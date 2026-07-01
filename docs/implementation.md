# Implementation notes (living)

What is **actually built and verified**, updated as each component lands. Companion
to — not a replacement for — [`../SYSTEM_DESIGN.md`](../SYSTEM_DESIGN.md)
(architecture), [`../ROADMAP.md`](../ROADMAP.md) (plan), and
[`../IMPLEMENTATION_PLAN.md`](../IMPLEMENTATION_PLAN.md) (TDD phases + open
decisions).

> **Keep this current.** When you complete a component, add/adjust its row here
> (contract, test, verification). Updating this file is part of a component's
> "done" (ROADMAP §1.4, plan R3).

## Status by phase

| Phase | State | Notes |
|---|---|---|
| **P0** — L0 core primitives | ✅ done (merged, PR #1) | + naming convention (`fsw`) |
| **P1** — Slate data model | 🚧 in progress | done: `core/util.h`, `AlignedBuffer`, `enum/SymbolTable`, `slate_enums`, `slate_id` (packed element ID). Next: `slate_type` (`slate_type_id<T>` name-hash + `slate_type_info_t`) → `slate_info<T>` trait → `SlateElement` → `SlatePathMap` (indices start at 1) → `SlateBuilderStore` → `SlateMemory` → compile `Slate`/`SlateCombiner` → golden-path integration test |
| P2+ | ⬜ not started | see ROADMAP |

**Build root:** `vehicle/` — includes resolve as `src/...` (Bazel `strip_include_prefix="/vehicle"`).
**Gate:** `./scripts/run_l0_tests.sh` (g++ `-Werror`) + Bazel; `./scripts/check.sh` is the authoritative wrapper.

## Implemented components

### L0 — core primitives (`vehicle/src/bullwinkle/all/`, `vehicle/src/hash/`)

| Component | Contract (what it is) | Test | Notes |
|---|---|---|---|
| `core/drone_types.h` | fixed-width `UINT8..64`/`INT8..64`/`uint` + `FSW_DISALLOW_COPY_AND_ASSIGN` | `drone_types_test.cc` | global scope (matches unqualified usage) |
| `core/fsw.h` | `FswAbortIfNot`/`FswAbortIf`/`FswAssert`/`FswDebugAssert` + `fsw_report` | `fsw_test.cc` (incl. pointer/int conditions) | `FswAbortIf` coerces via `!!(cond)`; **D24**: `FswAssert` prod semantics (abort → safe-state) TBD at P1 |
| `core/fswtime.h` | `nano_t`, `nano_t_min/max`, `billion`, `get_rel_time`, `fswsleep` | `fswtime_test.cc` | |
| `core/util.h` | `str_v`/`str_s`/`str_v_v`, `join`, `MonotonicPool` (bump arena `allocate(size,align)`+`release()`) | `util_test.cc` | `std::set/string` are **cold build-phase only** — keep out of hot paths |
| `hash/Hash128.h` + `hash/xxh.h` | `Hash128{u64[2]}`; `digest_xxh128(buf,len,seed)` deterministic 128-bit | `hash_test.cc` (determinism, sensitivity, avalanche) | impl is MurmurHash3-x64-128; **D23**: benchmark vs XXH3 before P7 per-cycle shard hashing |
| `B2.h` / `B2c.h` | mutable / const position-independent byte spans (`buf()`,`len()`; `B2→B2c`) | `b2_test.cc` | |
| `runtime.h` | `RUNTIME` / `INFRASTRUCTURE(...)` / `HOTSYNC_EXEMPT` annotation macros (no-ops) | `runtime_test.cc` (variadic `INFRASTRUCTURE`) | analyzer not wired; used nowhere yet |
| `static_vector.h` | fixed-capacity inline vector | `static_vector_test.cc` | `push_back` overflow always-checked; `operator[]` debug-only (`FswDebugAssert`, verified single-load in release); inline storage eager-constructs `N` (raw-storage deferred) |

### L1 — Slate (in progress)

| Component | Contract | Test | Notes |
|---|---|---|---|
| `core/AlignedBuffer.h` + `AlignedBuffer.cc` | per-shard heap storage: `ensure(amount,align)` geometric grow (32B-aligned, preserve + zero-fill), `clone()` deep copy, move-only | `aligned_buffer_test.cc` | the contiguous backing for `SlateMemory`'s `shard_table[num_slate_shard_t]` |
| `enum/SymbolTable.h` + `.cc` | bijective name↔`uint` reflection table: `add` (rejects either-side collision), `raw_get` (both directions), `get`→name/`""`, `dump`, `==`/`!=` | `SymbolTable_test.cc` | build-phase/diagnostic reflection (node-based std maps by design, **cold path only**); backs `slate_shard_t_sym`, `slate_elem_access_t_sym`, `state_sym`/`ctask_sym`, and the `SlateCombiner` cross-string enum-agreement check |
| `slate_enums.h` + `.cc` | `slate_shard_t` (7 shards + `shard_invalid`), `slate_elem_access_t` (ordered private<read_only<read_write), `slate_permission_t` bitmask + algebra (`deny`/`is_superset`/`take_subset`/`can_read`/`can_write`/`can_create`), `slate_shard_t_sym`/`slate_elem_access_t_sym` reflection | `slate_enums_test.cc` | permission = read/write/create + create-sync/create-nonsync qualifiers; sync-class shards = {sync, sync_no_telem} (the create-time boundary per SYSTEM_DESIGN); shard value is dense (array index + packed ID field) |
| `slate_id.h` | packed `slate_element_t` = `[offset:32][index:26][shard:4][w:1][v:1]`; `buildup`/`breakdown`/`index`/`can_write`/`has_validator`/`ro`/`is_valid`/`build_invalid`; `slate_element_default`=0 | `slate_id_test.cc` | **hot-path** — resolution is pure shift/mask (objdump: 0 calls, 0 branches under `-O2`). Bound ids are never 0 (layout indices start at 1); `build_invalid` carries `shard_invalid`. `slate_index_t`/`slate_offset_t` = UINT32 |
| `slate_type`, `slate_info<T>` trait, `SlateElement`, `SlatePathMap`, `SlateBuilderStore`, `SlateMemory`, `Slate`, `SlateCombiner` | — | — | pending (next P1 increments) |

## Verification status (current)

- **g++** `-Wall -Wextra -Werror`: **12/12** unit tests green (`scripts/run_l0_tests.sh`).
- **Bazel**: 12/12 (`//vehicle/src/bullwinkle/all:all`, `//vehicle/src/hash:all`, `//vehicle/src/bullwinkle/all/enum:all`).
- **valgrind**: leak/UB-clean on the memory-touching tests (`hash`, `b2`, `static_vector`, `util`/arena, `aligned_buffer`, `symbol_table`, `slate_enums`).
- **Perf spot-checks**: `static_vector::operator[]` is a single load under `-O2 -DNDEBUG` (bounds check compiles out); `slate_id` breakdown+index resolves in pure `mov`/`shr`/`and`/`add` (0 calls, 0 branches).
- ASan/UBSan runtime libs are absent in the CI sandbox; valgrind substitutes.

## Open flags / deferrals (see IMPLEMENTATION_PLAN §8)

- **D23** — `digest_xxh128` is MurmurHash3, not XXH3: benchmark shard-hash throughput vs. control period at P7; swap in XXH3 behind the same signature if it's a per-cycle bottleneck.
- **D24** — `FswAssert` always-aborts: reconcile with the fail-to-safe-state failure policy at P1.
- `static_vector` eager-constructs its `N` inline slots (requires default-constructible + copy-assignable `T`); acceptable while `T` is cheap and off the hot path — raw aligned storage deferred until a use site needs it.
- `core/util.h` `std::set`/`std::string` are node-based/heap containers, acceptable only as cold build-phase code — must not leak into a hot path.

## How to build & test

```bash
./scripts/check.sh                    # authoritative gate (g++ tests + best-effort bazel)
./scripts/run_l0_tests.sh             # g++ -Werror, all core/storage unit tests
bazel test //vehicle/src/bullwinkle/all:all //vehicle/src/hash:all --test_output=errors
```
