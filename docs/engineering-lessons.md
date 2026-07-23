# Engineering lessons (codified from review findings)

Binding guidance for coding agents working in this repo, distilled from what
systematic review actually caught during the P0–P1 build-out. Each rule exists
because its violation produced a **real, observed defect here** — the incident is
cited so the rule stays falsifiable. Read alongside the `data-oriented-design`
skill (the coding doctrine) — this file is the *process* doctrine.

## 1. A test that mirrors your inference proves nothing

**Incident:** `slate_enums` shipped reflection names without their `shard_`/`slate_`
prefixes and a wrong permission class. Unit tests passed — they asserted the same
wrong mental model the code implemented. Both were caught only by reading consumer
call sites (`SlateLayout` does `.substr(strlen("shard_"))`; `BasicControl` creates
cyclic state through the sync-only handle).

**Rule:** for every inferred contract, at least one assertion must mirror the
*consumer's exact expression*, not your restatement of it. The strongest gate is
compiling (then linking, then running) the real consumer — wire each imported `.cc`
into the gate the moment its includes resolve.

## 2. Verify doctrine claims at the API boundary, not the inner primitive

**Incident:** "the hot path is shifts+load" was objdump-verified for `slate_id`
(the inner primitive) — and stayed false for the *actual* API (`slate[token]`) for
weeks: a memoized `slate_type_id<T>()` guard ran per access (measured 3.3× tax) and
blocked inlining entirely at `-O2`.

**Rule:** performance/DOD claims are verified by measuring **through the entry
point callers use** (objdump the public call, count calls/branches; benchmark).
A clean inner layer proves nothing about the wrappers above it. Re-verify after
integration, not just at the layer's own PR.

## 3. Fail closed, everywhere, including "unreachable" defaults

**Incident:** `slate_can_create` classified unknown shards as freely creatable;
the classifier's fall-through returned the most permissive class. Harmless today
only because callers happened to range-check first.

**Rule:** in permission/safety code, every default, fall-through, and
out-of-range branch **denies**. "A caller already checks" is not a defense — the
next caller won't.

## 4. Prefer compiler-enforced invariants over tested ones

**What worked here:** stringize-from-token for reflection names (name cannot
drift from the symbol); exhaustive `switch` + `-Wswitch -Werror` (a new shard
fails to compile until classified); `static_assert` that bit-fields tile the
word; the ADL hook for auto-enums (missing reflection = compile error at the
instantiation site, not a silent fallback).

**Rule:** when an invariant *can* be made a compile error, do that first; a test
is the fallback, a comment is the last resort.

## 5. Imported code is contract truth, not quality truth

**Incidents:** `Slate.h` reused `SLATE_BUILDER_H` as its include guard — its
entire body was **silently skipped** when included (the "incomplete Slate"
mystery). Truncated files (missing `#endif`), return-type-only overloads,
`%hu` on a 64-bit id.

**Rules:** (a) treat imported code as the authority on *contracts* and restore
only objective defects (truncation, guard collisions, typos), each logged in the
commit; (b) when a header "compiles standalone but its symbols are missing,"
check the include guard *first*; (c) quarantine imported diagnostic quirks in a
relaxed warning tier — never lower the bar for owned code (owned = full
`-Wall -Wextra -Werror`; the gate's tiers encode this, `slate-internals.md` §10).

## 6. Design per-cycle operations for the steady state, at freeze time

**Incident:** `compute_shard_deltas` walked the whole element directory through
map nodes for every shard (86% skips), every frame — when the steady state of an
agreeing system is "no deltas at all" (one memcmp, measured ~90×).

**Rule:** per-cycle code gets (a) an early-out for the overwhelmingly common
case, and (b) dense, per-purpose structures **precomputed once at build/freeze**
— never iterate a build-phase directory at runtime.

## 7. Docs are load-bearing; drift is a defect, not a chore

**Incident:** `slate-internals.md` said `slate()` freezes the layout (it only
vends a handle; `build()` freezes) — a newcomer following the doc would write
real bugs. The golden-path test carried comments promising checks that didn't
exist.

**Rule:** a doc claim about behavior is a testable statement — when reviewing,
*diff docs against code* like code. Never write a comment promising coverage the
test doesn't have. Correcting the doc is part of the change that invalidated it
(ROADMAP §1.4 already binds this; reviews must enforce it).

## 8. Deduplicate cold-path scaffolding before it calcifies

**Incident:** three identical bounded-printf bodies, a verbosity knob no code
could turn, ~120 lines of never-called operators — accumulated in *weeks*, in
code written carefully.

**Rule:** at each phase boundary, sweep for: duplicated helper bodies, knobs
without setters, symbols with zero call sites (grep the whole tree, including
not-yet-gated imports before deleting — hold anything a pending consumer uses).

## 9. Edit mechanically, verify immediately

**Incident:** a greedy regex deletion in `Handle.h` ate the class body; the gate
caught it within one command because it runs after every edit batch.

**Rule:** prefer line-anchored or structure-aware edits over broad regexes; run
the full gate after **every** edit batch, not at the end; keep the gate's
failure output diagnosable (a red gate that prints nothing is the worst failure
mode for an autonomous loop — capture and print run output).

## 10. Review adversarially, in parallel, against a running system

**What worked:** three independent Fable-5 reviewers (complexity, performance,
interpretability), each grounded in the live golden-path harness, each required
to produce FILE:LINE findings with measurements — surfaced a measured hot-path
violation, a fail-open classifier, UB, and doc drift that self-review missed.

**Rule:** at phase boundaries, run parallel reviews split by *axis* (not by
file), demand measured evidence over opinion, and land fixes as separate
gate-verified commits: measured performance batch, simplification batch,
doc-reconciliation batch. Defer explicitly (with a tracking note) rather than
silently dropping findings whose consumers haven't landed.

## 11. The loop itself: review → extract → codify → apply (meta-rule)

This file is not a one-time artifact — it is the output of a **standing
iteration loop** that runs at every phase boundary and after any significant
post-merge defect:

1. **Review** — parallel adversarial reviews split by *axis* (complexity/
   modularity, measured performance, interpretability/doc-drift), grounded in
   the running gate, producing FILE:LINE findings with evidence (rule 10).
2. **Triage** — every finding lands in exactly one bucket: *fix now* (as
   separate gate-verified batches), *defer with a tracking note at the point of
   use*, or *reject with a stated reason*. Silent drops are forbidden.
3. **Extract** — for each fixed or deferred finding, ask: *"what rule would
   have prevented this class of defect?"* A finding becomes a lesson only if it
   is (a) a repeatable failure mode, not a one-off, and (b) expressible as a
   falsifiable, actionable rule.
4. **Codify** — amend THIS file. Format is binding: every rule cites the real
   incident that motivated it (so the rule stays falsifiable), and states the
   behavior to adopt. If an incident shows an *existing* rule failed to prevent
   a recurrence, strengthen that rule in place (and say why it failed) rather
   than adding a near-duplicate. If the rule list changes materially, refresh
   the one-line summary in CLAUDE.md so every future session loads it.
5. **Apply retroactively** — if the new lesson has a mechanical signature
   (a greppable/scriptable defect class, like the include-guard collision),
   sweep the entire tree for it immediately and record the result in
   `import-audit.md`.
6. **Close** — the lessons amendment ships in the same PR as the fixes it was
   distilled from, gate green. A review cycle that fixes code but writes no
   lessons (or vice versa) is incomplete.

**Trigger points:** every phase acceptance gate (P1, P2, …); any defect found
after merge that the existing rules should have caught; any review of ≥3
findings. The loop is bound into the plan (IMPLEMENTATION_PLAN §3 phase format
and §5b) and into CLAUDE.md, so no session can miss it.
