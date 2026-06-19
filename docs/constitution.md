# Constitution

> The **elves** Judge reads this every batch and verifies the system still keeps
> these promises. These are **deal-breakers** — if one is broken, you revert the
> whole PR without reading further. They exist to beat the *gaming problem*: when
> the agent writes both the code and the tests, it can satisfy the letter of a
> test while missing the point. The constitution gives the Judge success criteria
> the agent did **not** author and cannot narrow.
>
> The entries below are a **domain-grounded starting point** drawn from
> `SYSTEM_DESIGN.md` — review and replace with the promises that actually matter
> for the work you are launching. Keep each one *specific enough to verify*,
> *abstract enough to survive refactoring*, and *stated as a behavior, not an
> implementation*.

## Redundancy & agreement invariants
The properties that make three independent strings behave as one. (See
`SYSTEM_DESIGN.md` → "Keeping the three units in agreement.")

- Control logic runs **only when the supervisory layer reports synchronized**;
  no control output is computed or applied while a string is out of sync.
- After the input vote, all three strings act on an **identical, agreed-upon
  input set** — control never consumes an un-voted, per-string input.
- **Replicated (`sync`) state stays bit-identical** across strings: a string
  that diverges either recovers (hotsync) or is dropped from the vote — it never
  silently contributes divergent state.
- A failed/forged input is **rejected, not trusted**: inbound shared inputs that
  fail signature verification never enter the vote.

## State-model invariants
The rules that keep the Slate portable and the redundancy boundary intact.

- **No pointers are ever stored in the Slate** — only position-independent
  values (so bytes stay valid across machines and reboots).
- **Private (`nonsync`) state is never shared** between strings, and only the
  control logic may create **replicated (`sync`)** state. The creation-time
  permission boundary is never bypassed.
- Per-cycle scratch (`cyclic`) state is **wiped every frame**; no value is read
  across a frame boundary assuming it persisted.

## Safety & failure invariants

- Loss of **any single string never interrupts control** of the vehicle.
- A failure is **visible, not swallowed**: an error path aborts/flags rather than
  silently continuing with bad state.

## Process invariants

- The verification gate (`scripts/check.sh`) is **green** on the committed tip of
  every batch — no batch lands on a red build or a skipped/disabled test.
- No secret, key, or credential is ever committed to the repo or written to a log.
