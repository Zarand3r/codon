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
| **P1** — Slate data model | 🚧 in progress | done: type system (`SymbolTable`, `slate_enums`, `slate_id`, `slate_type`, `slate_info<T>`), fsw logging+verbose layer, `Handle`, `AlignedBuffer`, `SlateElement`+`SlatePathMap`. Next: `slate_accessor`/`Signal`/`slate_validator_t` + `.enum.h` shims → `SlateBuilderStoreInterface` → `SlateMemory` → `EnumRegistry` → `SlateLayout.cc`/`SlateBuilder.cc`/`Slate.cc` compile → golden-path (L1) integration test. **Internals: see [slate-internals.md](slate-internals.md).** |
| P2+ | ⬜ not started | see ROADMAP |

**Build root:** `vehicle/` — includes resolve as `src/...` (Bazel `strip_include_prefix="/vehicle"`).
**Gate:** `./scripts/run_l0_tests.sh` (g++ `-Werror`) + Bazel; `./scripts/check.sh` is the authoritative wrapper.

## Implemented components

### L0 — core primitives (`vehicle/src/bullwinkle/all/`, `vehicle/src/hash/`)

| Component | Contract (what it is) | Test | Notes |
|---|---|---|---|
| `core/drone_types.h` | fixed-width `UINT8..64`/`INT8..64`/`uint` + `FSW_DISALLOW_COPY_AND_ASSIGN` | `drone_types_test.cc` | global scope; **64-bit types are the `long long` family** (`static_assert` sizeof==8) so imported `%llx`/`%lld` format specifiers are warning-clean |
| `core/fsw.h` + `core/fsw_log.h` | asserts (`FswAbortIfNot`/`FswAbortIf`/`FswAssert`/`FswDebugAssert` + `fsw_report`); **logging/context layer** (`dbnprintf`/`dbstring`/`FswPrefix`/`FswStackFrame`, typed `FswAbortIfEqInt/EqUint64/Neq/NeqInt/NeqInt64/NeqDouble/OpUint64`, `FswMsgAbortIf`/`FswMsgAbortIfNot`, `FswIf`/`FswIfNot`/`FswIfNeq`); **verbose diagnostics** (`report_abort`, `FswArg`, `FswOnVerbose`, `fsw_verbose` gate) | `fsw_test.cc`, `fsw_log_test.cc` | `fsw.h` includes `fsw_log.h`; split keeps `<string>`/`<cstdarg>` out of the assert core. `FswStackFrame::get_current_stack_frame()` returns null (no producer yet). **D24** still open |
| `Handle.h` (imported) | `Handle<T>` shared-ownership smart pointer (`std::shared_ptr`-backed): ctor/`assume_ownership`/`get`/explicit `bool`/`*`/`->`/`is_unique`/compare/`assign_casted` | `handle_test.cc` | now compiles (needed the verbose fsw primitives above); used everywhere (`Handle<SlateMemory>`, `Handle<SlateBuilderStoreInterface>`, …) |
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
| `slate_enums.h` + `.cc` | `slate_shard_t` (7 shards + `shard_invalid`), `slate_elem_access_t` (ordered private<read_only<read_write), `slate_permission_t` bitmask + algebra (`deny`/`is_superset`/`take_subset`/`can_read`/`can_write`/`can_create`), `slate_shard_t_sym`/`slate_elem_access_t_sym` reflection | `slate_enums_test.cc` | permission = read/write/create + create-sync/create-nonsync qualifiers; **create classes: sync={sync,sync_no_telem} needs c_sync, nonsync={nonsync,nonsync_no_telem} needs c_nonsync, static/cyclic need only the create bit** (control creates cyclic scratch via its sync-only handle — BasicControl); sym **names carry `shard_`/`slate_` prefixes** (SlateLayout does `.substr(strlen("shard_"/"slate_"))`); shard value is dense (array index + packed ID field) |
| `slate_id.h` | packed `slate_element_t` = `[offset:32][index:26][shard:4][w:1][v:1]`; `buildup`/`breakdown`/`index`/`can_write`/`has_validator`/`ro`/`is_valid`/`build_invalid`; `slate_element_default`=0 | `slate_id_test.cc` | **hot-path** — resolution is pure shift/mask (objdump: 0 calls, 0 branches under `-O2`). Bound ids are never 0 (layout indices start at 1); `build_invalid` carries `shard_invalid`. `slate_index_t`/`slate_offset_t` = UINT32 |
| `slate_type.h` | `slate_type_t` (UINT64), `slate_type_invalid`=0, `slate_type_id<T>()` — hash of `__PRETTY_FUNCTION__` per-type name via `digest_xxh128`, folded to 64b, never 0 | `slate_type_test.cc` (stable, distinct incl. look-alike structs, template instantiations) | cross-string-stable (same binary → same name → same id); cold (build-time tag), memoized per type |
| `slate_info.h` | umbrella header (aggregates `slate_id` + `slate_type` + `core/fsw`) **+ the `slate_info<T>` value trait** — typed views `R`/`W`/`O`, `size(value)`/`alignment`/`type_id`, `is_valid` gate, `construct`/`copy`/`from_mem` | `slate_info_test.cc` (behavioral: construct→from_mem→copy round-trips in aligned memory) + `slate_tokens_compile` gate | primary template = trivially-copyable POD/enum (fixed size); `is_valid` rejects pointers & non-trivially-copyable (std::string); variable-size `Slice<>` specialization lands with that type |
| `slate_tokens.{h,cc}` (consumer-compile) | **gate, not owned here**: the imported token layer + accountant impl compiles against `slate_info` | `slate_tokens_compile_test.cc` + `slate_tokens.cc`, object-only (§4) | **now the full `.cc` compiles** (fsw logging layer landed); caught two real contract issues (`FswIfNeq` missing; `UINT64`→`long long` for `%llx`) |
| `SlateElement.h` (`SlateElementMetadata`) | per-element layout record: `type_id`/`shard`/`value_offset`/`value_size`/`access_policy`/`subsystem_id`/`is_view_element` (aggregate of trivial fields) | via `slate_path_map_test.cc` | frozen at build; the element's packed id is built from `shard`+`value_offset`+the map index |
| `SlatePathMap.h` | element directory: `std::map<string_view, metadata>` (pool-interned keys) + dense **1-based** index bijection (`insert`→`{it,inserted}`, `iterator_to_id`/`id_to_iterator`, `find`/`clear`) | `slate_path_map_test.cc` (behavioral: 1-based indexing, inverse round-trip, dup-reject, index-0 reserved) | **enforces the `slate_id` index-≥-1 invariant** (index 0 reserved so a bound id ≠ default); keys don't own the chars (SlateLayout's pool does) |
| `slate_{shard_t,elem_access_t,subsystem_id_t}.enum.h` | compatibility shims: imported consumers include per-enum `.enum.h` (dropped codegen). shard/access re-export `slate_enums.h`; subsystem_id is the canonical home of `slate_subsystem_id_t` (moved out of `SlateElement.h`) | `slate_enum_headers_compile_test.cc` (object-only) | unblocks include resolution for `SlateLayout.h`/`StateMachine`/`CurlFileDownload`/`DroneGncControl` |
| `slate_accessor.h` | validator/accessor subsystem: `SlateValidator` (polymorphic base), `SlateTypedValidator<T>` (`validate(const T&, T&)`), `SlateFunctionValidator<T>`, `function_validator<T>` factory, `slate_validator_t` (`internal`/`is_noop`/`is_usable`), `SlateAccessor<T>` (read-convert, `store()` validates, `operator=`) | `slate_accessor_test.cc` (behavioral: accept/reject leaves value unchanged, wrong-type down-cast fails cleanly) | **polymorphic design dictated by consumer** (SlateBuilder binds via `Handle::assign_casted` = `dynamic_pointer_cast`); `enum_validator` deferred (no real callers — doc-comment only); validators heap-held at build, runtime store = 1 virtual call |
| `slate_type` registry | `slate_type_info_t {id, name}` + `slate_type_info_utils::get_type_info(id, info)`; `slate_type_id<T>()` registers `{id, name}` on first use (id→name for diagnostics) | `slate_type_test.cc` | id stays the 64-bit hash (order-independent → cross-string-safe; a 16-bit id would collide); `%hu` in imported error strings prints the low 16 bits (lossy diagnostic, not correctness) |
| `slate_path.h` | `validate_slate_path` (segments `[A-Za-z0-9_]`, single `.` separators), `slate_join_path`, `slate_rel_path` (both `string_view`/`string` out) | via `SlateLayout.cc` consumer-compile | dot-separated hierarchical paths; cold (build-time only) |
| `SlateLayout.{h,cc}` (**compiles**, object-only) | the layout/allocation engine: `allocate_element` (carve aligned shard space, intern path, assign 1-based index, build packed id), `create_view_element`, freeze, validator registry | `SlateLayout.cc` relaxed consumer-compile (§4) | **largest Slate consumer now type-checks against the whole foundation.** Added missing `B2`/`B2c`/`slate_path` includes; relaxed `-Wformat`/`-Wunused-parameter` on its imported diagnostic quirks |
| `SlateBuilderStore`, `SlateMemory`, `EnumRegistry`, `Slate`, `SlateCombiner` | — | — | pending (next P1 increments) |

## Verification status (current)

- **g++** `-Wall -Wextra -Werror`: **17 unit tests + 2 consumer-compiles** green (`scripts/run_l0_tests.sh`).
- **Bazel**: all green (`//vehicle/src/bullwinkle/all:all`, `//vehicle/src/hash:all`, `//vehicle/src/bullwinkle/all/enum:all`); `slate_tokens_compile` (now incl. `slate_tokens.cc`) is a compile-only `cc_library` consumer gate.
- **valgrind**: leak/UB-clean on the memory-touching tests (`hash`, `b2`, `static_vector`, `util`/arena, `aligned_buffer`, `symbol_table`, `slate_enums`).
- **Perf spot-checks**: `static_vector::operator[]` is a single load under `-O2 -DNDEBUG` (bounds check compiles out); `slate_id` breakdown+index resolves in pure `mov`/`shr`/`and`/`add` (0 calls, 0 branches).
- ASan/UBSan runtime libs are absent in the CI sandbox; valgrind substitutes.

## Open flags / deferrals (see IMPLEMENTATION_PLAN §8)

- **D23** — `digest_xxh128` is MurmurHash3, not XXH3: benchmark shard-hash throughput vs. control period at P7; swap in XXH3 behind the same signature if it's a per-cycle bottleneck.
- **D24** — `FswAssert` always-aborts: reconcile with the fail-to-safe-state failure policy at P1.
- `static_vector` eager-constructs its `N` inline slots (requires default-constructible + copy-assignable `T`); acceptable while `T` is cheap and off the hot path — raw aligned storage deferred until a use site needs it.
- `core/util.h` `std::set`/`std::string` are node-based/heap containers, acceptable only as cold build-phase code — must not leak into a hot path.
- ~~fsw logging layer missing~~ **RESOLVED** — `core/fsw_log.h` now provides `dbnprintf`/`dbstring`/`FswPrefix`/`FswStackFrame` + the typed/msg/`FswIf*` macro family; `slate_tokens.cc` compiles in full. `FswStackFrame` has no frame producer yet (returns null; a future enhancement can push frames from the abort macros).

## How to build & test

```bash
./scripts/check.sh                    # authoritative gate (g++ tests + best-effort bazel)
./scripts/run_l0_tests.sh             # g++ -Werror, all core/storage unit tests
bazel test //vehicle/src/bullwinkle/all:all //vehicle/src/hash:all --test_output=errors
```
