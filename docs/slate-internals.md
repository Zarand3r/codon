# Slate internals (full detail)

The **Slate** is Codon's single, named model of all vehicle state. This document
explains how it actually works internally — the memory model, the identifier
encoding, the type system, the build-vs-runtime split, and the invariants that make
the triple-redundant framework possible. It is the deep companion to
[`../SYSTEM_DESIGN.md`](../SYSTEM_DESIGN.md) (architecture) and
[`implementation.md`](implementation.md) (what's built + verified).

> Scope: this describes the Slate *data model* (ROADMAP L1 / plan P1). Sharing,
> voting, time-sync, and recovery build **on top of** these internals and are
> covered in SYSTEM_DESIGN.

---

## 1. What the Slate is, in one picture

```
                         ┌──────────────────────────── Slate ───────────────────────────┐
   typed API (per T)     │   load<T>(token) / store<T>(token,v) / slate[token]            │
   ── compile time ──    │                    │ resolves token.id                         │
                         │                    ▼                                           │
   packed id (64-bit)    │   slate_element_t = [offset:32][index:26][shard:4][w:1][v:1]   │
                         │                    │ shard + offset (shifts/masks, hot path)   │
                         │                    ▼                                           │
   value bytes (heap)    │   SlateMemory::shard_table[shard]  (AlignedBuffer, 32B-aligned)│
                         │        └── flat, contiguous, POINTER-FREE byte image ──────────│
                         │                                                                │
   schema (build-time)   │   SlateLayout: SlatePathMap (path→SlateElementMetadata)        │
                         │                + initial-value templates + validators          │
                         └────────────────────────────────────────────────────────────────┘
```

There are **four distinct representations of "an element"**, each for a different
job — keeping them separate is the whole design (see §7):

| Representation | Lives in | Phase | Holds the value? |
|---|---|---|---|
| **Typed token** (`WriteToken<T>`) | caller's object | compile+build | no — carries a packed id |
| **Packed id** (`slate_element_t`) | inside the token | built once, frozen | no — `shard`+`offset`+`index`+flags |
| **Metadata** (`SlateElementMetadata`) | `SlatePathMap` in `SlateLayout` | build, then frozen | no — schema (type/offset/size/policy) |
| **Value bytes** | `SlateMemory::shard_table[shard]` | mutated every cycle | **yes** |

---

## 2. The memory model: shards as flat, pointer-free byte images

All live state lives in **shards** — there are exactly **7**, enumerated by
`slate_shard_t` (`slate_enums.h`):

| Shard | Replicated/voted? | Lifetime | Telemetered? |
|---|---|---|---|
| `shard_static` | no | immutable after build | yes |
| `shard_sync` | **yes (voted across strings)** | persists | yes |
| `shard_nonsync` | no (string-private) | persists across frames | yes |
| `shard_cyclic` | no | **reset every frame** | yes |
| `shard_sync_no_telem` | yes | persists | no |
| `shard_nonsync_no_telem` | no | persists | no |
| `shard_cyclic_no_telem` | no | reset every frame | no |

Each shard's live bytes are one `AlignedBuffer` — a single **heap** block,
**32-byte aligned**, held inline in `SlateMemory::shard_table[num_slate_shard_t]`.
Every element's value sits at a fixed byte `offset` inside its shard's block. A
read/write is therefore:

```cpp
shard_table[shard].data() + offset      // → the value's address
```

Four properties are enforced or guaranteed on that block, and each buys a
framework capability:

- **Flat / contiguous** — one array, values at fixed offsets, no nodes to chase.
  *Enables:* `memcpy`/hash/compare of the whole shard.
- **Aligned** — the buffer is 32B-aligned and each `value_offset` is rounded up to
  `alignof(T)` by the allocator (`(size + a - 1) & -a`). *Enables:* tearing-free,
  fault-free, SIMD-friendly access.
- **Pointer-free** — the bytes contain **no addresses**; values are scalars, POD
  aggregates of scalars, or inline `[len][bytes]` slices — never a pointer to
  storage elsewhere. *Enables:* position-independence (below).
- **Position-independent** — because there are no pointers, the raw bytes are
  meaningful in *any* process/machine/run. *Enables:* ship a shard over a datagram
  and deposit it directly into a peer (`swap_shard_buffer`), byte-for-byte equality
  (`compute_hash`), single-element diffs (`compute_shard_deltas`), and reload after
  reboot — with **no serialization**.

**What enforces pointer-freeness** (see §4.3): `slate_info<T>::is_valid()` rejects
pointers and non-trivially-copyable types at `create<T>` time; a trivially-copyable
struct that *contains* a pointer cannot be caught generically in C++17, so that last
mile is a documented structural rule plus a grep gate (plan P1 property test).

---

## 3. The packed identifier — `slate_element_t`

A token carries a 64-bit opaque id that encodes everything the hot path needs
(`slate_id.h`):

```
bit 63 ............................................. bit 0
[   offset : 32   ][  index : 26  ][ shard : 4 ][ w:1 ][ v:1 ]
  byte offset        element index    slate_       may_   has_
  into shard (≤4GiB) (≤~67M elems)    shard_t      write  validator
```

- Resolution is pure `shr`/`and`/`add` — `objdump`-verified **0 calls, 0 branches**
  under `-O2`. No lookup, no string compare at runtime.
- `slate_id_buildup(...)` packs and **rejects out-of-range fields** (incl. index 0);
  `slate_id_breakdown(...)` unpacks the load/store fields; `slate_id_index()`,
  `slate_id_can_write()`, `slate_id_has_validator()`, `slate_id_ro()` (clear write
  bit) are single-field ops.

### Validity & sentinels (a subtle, load-bearing detail)

- `slate_element_default == 0` is the "no element" sentinel.
- **A bound id is never 0** because element **indices start at 1** — index 0 is
  reserved. This is enforced *structurally* at the single mint point:
  `slate_id_buildup` rejects index 0, and `SlatePathMap` (the only index producer)
  assigns 1-based indices. So `slate_id_is_valid(id) = (id != 0 && shard < num)`.
- Un-bound placeholder ids (`slate_id_build_invalid(count)`, handed out by the token
  accountant before binding) carry `shard_invalid`, so they are never valid, never
  the default, and disjoint from every bound id.

The field widths (64-bit; 32/26 split) were a human-confirmed design decision;
`static_assert`s in `slate_id.h` verify the fields tile the word exactly.

---

## 4. The type system: tokens, `slate_type_id`, and the `slate_info<T>` trait

### 4.1 Tokens (`slate_tokens.h`)

`ReadToken<T>` / `WriteToken<T>` / `WriteValidatorToken<T>` are typed handles.
Access rights are template parameters checked at compile time (`can_read`,
`can_write`, `can_validate`). A token holds only a `slate_element_t id`. The token
*accountant* tracks tokens created at init so the builder can detect an
uninitialized token that would crash at runtime.

### 4.2 Stable type id (`slate_type.h`)

`slate_type_id<T>()` returns a 64-bit id that is **identical across all three
strings** (they compare/serialize type ids while agreeing on layout). It is the
hash (`digest_xxh128`) of the compiler's per-type `__PRETTY_FUNCTION__` name, folded
to 64 bits, never 0. Same binary → same name → same id; distinct types → distinct
ids. Cold path, memoized per type. (`typeid().hash_code()` is *not* stable, so it
can't be used.)

### 4.3 The value trait (`slate_info<T>`)

The bridge between the typed API and the untyped byte model. It is a **compile-time
type trait** (only static members + typedefs — no runtime object; "instantiated" by
the compiler wherever `create<T>`/`store<T>`/`WriteToken<T>` names a concrete `T`).

| Member | POD meaning | Why it exists |
|---|---|---|
| `R` / `W` / `O` | `const T&` / `T&` / `T&` | what `load` returns, `store` takes, out-params use |
| `size(value)` | `sizeof(T)` | **takes a value** so variable-size `Slice<>` can report its runtime byte length |
| `alignment()` | `alignof(T)` | shard placement |
| `type_id()` | `slate_type_id<T>()` | element tagging + cross-string agreement |
| `is_valid()` | `is_trivially_copyable && !is_pointer` | **the gate** keeping non-position-independent types out |
| `construct(void*, R)` | placement-new | build-time write into the template |
| `copy(R, W)` | assignment | runtime overwrite of a live value |
| `from_mem(void*)→W`, `from_mem(const void*)→R` | typed view over bytes | turn a shard address into a typed reference |

**Why a trait and not raw `sizeof`/`alignof`:** for `double` the trait *is* trivial
and adds nothing. It earns its keep on (a) **variable-size** types (`size` depends on
the value), (b) types whose **in-shard bytes ≠ in-hand value** (a `Slice`'s inline
form, views into the buffer), and (c) being the **single admission gate** — using
`sizeof` directly would happily store a `std::string`, whose bytes hold a heap
pointer, silently breaking position-independence and cross-string hashing.

**Trait vs metadata (they are not redundant):** `slate_info<T>` is per-*type*,
compile-time, and needs `T` in hand. The runtime Slate core (`SlateMemory`,
`SlateCombiner`, `roll_frame`, telemetry) looks elements up by id/path and **has no
`T`** — so the facts it needs (`value_size`, `type_id`, `shard`, `offset`,
`access_policy`) must be captured as plain data at `create<T>` time. That captured,
type-erased, per-element data is `SlateElementMetadata`. The build phase is the only
place both are in scope: it *consumes* the trait to *produce* the metadata.

---

## 5. The schema: `SlateElementMetadata` and `SlatePathMap`

- **`SlateElementMetadata`** (`SlateElement.h`) — the frozen, per-element layout
  record: `type_id`, `shard`, `value_offset`, `value_size`, `access_policy`,
  `subsystem_id`, `is_view_element`. Pure description; it does **not** hold the value
  (only where and how big it is).
- **`SlatePathMap`** (`SlatePathMap.h`) — the element directory:
  `std::map<std::string_view, SlateElementMetadata>` whose keys are `string_view`s
  into `SlateLayout`'s interned path pool (the map does not copy the characters),
  **plus a dense, stable, 1-based index bijection**:
  - `insert(pair) → {iterator, inserted}` — a fresh path gets the next 1-based index;
  - `iterator_to_id(it) → slate_index_t` and `id_to_iterator(idx) → const_iterator`
    are inverses;
  - the 1-based rule is exactly the `slate_id` index-≥-1 invariant (§3).

Paths are hierarchical strings (`"nav.altitude_m"`), interned once in a
`MonotonicPool` bump arena so the map keys are cheap, stable `string_view`s.

---

## 6. Lifecycle: build phase, then frozen

The Slate has two strict phases (a core SYSTEM_DESIGN property).

### 6.1 Build phase (mutable structure)

`SlateBuilder::create<T>(path, initial_value, shard, access, token)`:

1. `slate_info<T>::is_valid()` — reject types that can't live in a shard.
2. compute `type_id = slate_type_id<T>()`, `value_size = slate_info<T>::size(value)`,
   `alignment = slate_info<T>::alignment()`.
3. `store->allocate_element(...)` (→ `SlateLayout`): carve aligned space in the
   shard's *initial-value template*, intern the path, `SlatePathMap::insert` to get
   the 1-based index, build the packed `slate_element_t`, and return `(element_id,
   mem)` where `mem` points into the template buffer.
4. `slate_info<T>::construct(mem, initial_value)` — **the build-time write**: places
   the initial value into the template (not a live Slate yet).
5. bind the token to `element_id`.

Permissions gate creation: `slate_can_create(permission, shard)` — only the control
string may create `sync` (voted) state; the control string is barred from `nonsync`
(string-private) state; `static`/`cyclic` need only the general create right (see
`slate_enums.h`, and the create-class classifier that a new shard must extend or the
build fails).

### 6.2 Freeze + materialize

Two distinct calls (a common misreading — `slate()` does NOT freeze):

- `builder.slate(validator_fn)` merely **vends a `Slate` handle** over the shared
  memory (and wires the optional validator signal). Valid to call before build;
  the handle only works once `build()` succeeds.
- `builder.build()` → store `finalize()` + token-accountant check +
  `SlateMemory::build()`: the layout is **frozen** (no more `create`;
  structure/offsets/type tags immutable), and each shard's initial-value template
  is copied into the live `shard_table[shard]` `AlignedBuffer`. The cyclic shards
  also keep a template that `roll_frame()` re-copies every frame.

### 6.3 Runtime (values mutate, structure does not)

- `slate.store<T>(id, v)` / `slate[wtoken] = v` → `slate_info<T>::copy(v,
  load_rw<T>(id))`, and `load_rw` resolves to `shard_table[shard].data() + offset`.
  **The runtime write.**
- `slate.load<T>(id)` / `slate[rtoken]` → `slate_info<T>::from_mem(...)` over the
  same address, returning a typed reference into the shard.
- For **unvalidated** elements: no allocation, no lookup, no string compare on
  this path — just the packed-id shifts and a fixed-offset memory access (plan
  property **P17**, `objdump`-verified: 0 calls, fully inlined).
- **Caveat — validated elements** (`WriteValidatorToken`): the current wiring
  resolves the validator per store (a map lookup + `dynamic_pointer_cast`/RTTI +
  shared_ptr atomics; measured ~16× an unvalidated store). The bind-time
  typed-pointer-caching fix is a tracked P1 hot-path item (see the HOT-PATH NOTE
  in `slate_accessor.h`); no validated element is on a control loop yet.

**All heap allocation happens during build/init only.** At flight time the Slate is a
pre-allocated (and, at P11, `mlockall`-pinned) arena the control loop reads/writes by
offset — never allocating, so latency is bounded.

---

## 7. Why the four representations are separate (design rationale)

- **Packed id vs metadata:** the id is the hot, 64-bit handle (only what a load
  needs, and it's physically full — `type_id` alone is 64 bits). Metadata is the
  cold, complete schema the build phase and diagnostics need. Splitting keeps the hot
  path a few shifts and the schema arbitrarily rich.
- **Metadata vs value:** metadata is frozen, shared-structure, identical across
  strings; the value mutates every cycle. If the value lived inside the metadata
  (`std::map` nodes with `string_view` keys), state would be scattered across heap
  nodes with pointers — no hashing, no `memcpy` sharing, no voting.
- **Trait vs metadata:** compile-time per-type knowledge vs runtime per-element,
  type-erased data — the runtime has no `T`, so it must read metadata (§4.3).

The throughline: **state is bytes.** The Slate goes to great lengths (pointer-free
shards, packed ids, type-erased metadata, a compile-time trait that gates types) so
that the redundancy framework can treat all vehicle state as a flat byte image it can
hash, diff, ship, vote on, and reload — deterministically and without serialization.

---

## 8. Where each piece lives in the tree

| Concern | File(s) |
|---|---|
| Shards / access / permissions | `slate_enums.h` (+ `.cc` for `_sym`) |
| Packed id | `slate_id.h` |
| Stable type id | `slate_type.h` |
| Value trait + umbrella | `slate_info.h` |
| Tokens + accountant | `slate_tokens.{h,cc}` |
| Element metadata + directory | `SlateElement.h`, `SlatePathMap.h` |
| Per-shard heap storage | `core/AlignedBuffer.{h,cc}` |
| Path interning arena | `core/util.h` (`MonotonicPool`) |
| Enum reflection | `enum/SymbolTable.{h,cc}` |
| Smart pointer | `Handle.h` |
| Layout engine (build/allocate/freeze) | `SlateLayout.{h,cc}` |
| Live memory (materialize/hash/deltas/roll) | `SlateMemory.{h,cc}` |
| Store (where a builder's data lives) | `SlateBuilderStore.{h,cc}` |
| Typed API surface | `Slate.{h,cc}`, `slate_accessor.h` (validators/accessor) |
| Enum registration | `EnumRegistry.h` |

Verified status of each is tracked in [`implementation.md`](implementation.md).

---

## 9. Fsw macro glossary (read before touching call sites)

The `Fsw*` names are pinned by the imported consumers and are **not** all asserts:

| Family | Behavior | Trap to know |
|---|---|---|
| `FswAbortIf(cond, ret)` / `FswAbortIfNot` / typed variants (`EqInt`, `NeqUint64`, `OpUint64`, …) | log, then **`return ret;` from the enclosing function** | "Abort" means *abort the function*, not the process — a hidden `return` inside a macro |
| `FswMsgAbortIf`/`FswMsgAbortIfNot`/`FswMsgAbort` | same hidden `return`, with a bounded formatted message | statement macros; can't be used in expressions |
| `FswIf(c)` / `FswIfNot(c)` / `FswIfNeq(a,b)` / `FswOutsideRange(x,lo,hi)` | **expressions**: the (unlikely-hinted) condition itself | no side effects; caller does the logging/returning |
| `FswAssert(cond)` | log + **`std::abort()`** (also in release) | the only genuinely fatal one |
| `FswDebugAssert(cond)` | `FswAssert` in debug, no-op under `NDEBUG` | never rely on its side effects |

## 10. Imported vs. owned (who is held to what)

The gate (`scripts/run_l0_tests.sh`) encodes three trust tiers:

- **Owned** (written here, full `-Wall -Wextra -Werror`, unit-tested): everything
  in §8 except the imported files below — the type system, path map, accessor,
  store, memory impl, fsw layer, tests.
- **Imported, restored** (reference truth for contracts; compiled with
  `-Wno-format -Wno-unused-parameter` for pre-existing diagnostic quirks; edited
  only to fix objective truncation/typo defects, each logged in commits):
  `Slate.{h,cc}`, `SlateBuilder.{h,cc}`, `SlateLayout.{h,cc}`, `SlateMemory.h`,
  `slate_tokens.{h,cc}`, `slate_accessor.h` doc block, `Handle.h`, `Signal.h`.
- **Consumer-compile gates**: imported `.cc` compiled object-only so a
  mis-inferred contract *fails to build* rather than passing a mirror test.
