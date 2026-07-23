# CLAUDE.md — Codon flight-control stack

Codon is a fault-tolerant, triple-redundant ("three strings" a/b/c) real-time
flight-control stack (the `bullwinkle` framework + the `flight` runtimes built on
it), built with Bazel. Start from [`SYSTEM_DESIGN.md`](SYSTEM_DESIGN.md) and
[`IPC_DESIGN.md`](IPC_DESIGN.md) for the architecture.

Agent setup (skills library + autonomous harness) is documented in
[`docs/agent-harness-setup.md`](docs/agent-harness-setup.md).

## Skills — use these automatically

The skills below come from the **`eng-skills`** plugin (the `claude-skills`
marketplace; auto-installed via [`.claude/settings.json`](.claude/settings.json)).
This routing table is loaded into context every session; the skill bodies load
only when Claude routes to them. Skills are invoked namespaced (e.g.
`/eng-skills:elves`) but Claude also auto-invokes them by description.

**Before and during any coding work, load the skill(s) whose trigger matches the
task** and follow their guidance. `karpathy-guidelines` applies to essentially
all coding; the others are routed to by task type. When in doubt, start with
`principal-production-engineer` — it is the single entry point that routes to the
rest.

| Skill | Load it when… |
|---|---|
| **karpathy-guidelines** | Always, for any writing/reviewing/refactoring of code. Avoid overcomplication, make surgical changes, surface assumptions, define verifiable success criteria. |
| **principal-production-engineer** | Implementing, reviewing, refactoring, or hardening production code in any language. Single entry point — enforces simple design, dense data, explicit ownership, visible failure, minimal abstraction, honest verification, pipeline discipline. Routes to the skills below. |
| **strategic-engineering-planner** | *Before* implementation when work is architecturally significant, ambiguous, multi-file, distributed, performance-sensitive, concurrency-heavy, or likely to need multiple passes. Produces a written roadmap first. Skip for trivial fixes and obvious CRUD. |
| **implementation-plan** | *After* the design is locked, *before* code. Turns a design doc into a checklist-first `IMPLEMENTATION_PLAN.md` with vertical-slice steps and binary acceptance gates. |
| **cpp-systems-internals** | Writing or reviewing C++ where hardware behavior, codegen cost, ownership vocabulary, API style, or kernel paging matters (lambdas, templates, cache lines, vtables, smart pointers/spans/arenas, `mmap`/`madvise`, AoS/SoA). Load only the relevant topic file. **This repo is exactly that kind of C++** — reach for it often. |
| **data-oriented-design** | **The mandatory coding-style doctrine for this repo.** Load it for *any* implementation here — this is a hard-real-time, triple-redundant flight controller whose Slate is a pointer-free, fixed-layout, byte-comparable model. Enforces: all state in the Slate, no pointers in the model, no hidden allocation/throwing/blocking/dispatch in `RUNTIME` hot-path methods, fixed-capacity/SoA/dense layout, branchless hot paths, ownership-in-types, and measure-first verification. Routes into `cpp-systems-internals` for C++ mechanism depth. See `ROADMAP.md` §1 and `IMPLEMENTATION_PLAN.md` §0 for how it binds here. |
| **auto-research** | Iteratively optimizing a measurable outcome unattended/overnight — loss, latency (p50/p95/p99), throughput, MFU, memory/binary size, compile time. Enforces a fixed eval harness, append-only results log, keep-on-improvement / reset-on-regression. |
| **elves** | Executing a *development plan* unattended/overnight — "run overnight," "implement this plan," "keep going without me," "I'll be back in the morning." Breaks the plan into sprint-sized batches, implements with tests + PR-based review, keeps durable memory for compaction recovery. Requires `git` + `gh`. |

**How to apply:** for a non-trivial task, the default flow is
`strategic-engineering-planner` (plan) → `implementation-plan` (checklist) →
`principal-production-engineer` (implement, routing into `data-oriented-design`
for the coding style and `cpp-systems-internals` for C++ mechanism depth), with
`karpathy-guidelines` governing throughout. **In this repo, load
`data-oriented-design` for every implementation** — its rules are the binding
coding standard (`IMPLEMENTATION_PLAN.md` §0/§1). For
unattended/overnight runs pick by goal: **`auto-research`** when success is *one
number on a fixed harness*, **`elves`** when success is *a development plan with
test/PR gates*. Read a skill's `SKILL.md` before acting on its domain.

## Build & verify

- Build/test gate: [`scripts/check.sh`](scripts/check.sh) (wraps Bazel — the
  repo's toolchain, pinned to `.bazelversion`). The autonomous harness runs this
  after every batch and will not advance on a non-zero exit.
- Constitution (ungameable promises the elves Judge enforces):
  [`docs/constitution.md`](docs/constitution.md).
- **Docs as you go:** [`docs/implementation.md`](docs/implementation.md) is the
  living record of what's actually built + verified. Update it whenever a component
  lands — it is part of "done" (ROADMAP §1.4). Design lives in `SYSTEM_DESIGN.md`,
  the plan in `ROADMAP.md`/`IMPLEMENTATION_PLAN.md`, naming in `docs/NAMING.md`.

## Naming conventions

This codebase descends from a SpaceX/Starlink slice — **no SpaceX/Starlink-alluding
names**. Flight-software house utilities/macros use the **`fsw`** prefix:
`FswAbortIfNot` (early-return guards) / `FswAssert` (true assert) — `core/fsw.h`; glossary in `docs/slate-internals.md` §9, `fswtime`/`fswsleep`,
`FSW_DISALLOW_COPY_AND_ASSIGN`, `fsw_*` socket/perf helpers. **Never reintroduce
`sx` / `SX` / `Sac` / `noc`.** Satellite-domain terms (`sat*`, `SAPC`, `Ttc`,
`Cola`, `Satellites::Fleet`) are replaced as the vehicle layer is reimplemented for
the drone pivot — prefer neutral (`vehicle`) or the agreed drone term, and surface
a design decision if unsure. Full rules + substitution table:
[`docs/NAMING.md`](docs/NAMING.md).
