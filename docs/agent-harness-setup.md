# Agent setup — skills library + autonomous harness

How this repo is wired for AI coding agents, and the one-time steps to finish
turning it on. The canonical, fuller documentation lives in the **claude-skills**
repo; this file records what was done here and points at it.

- Library overview & install options — `claude-skills/README.md`
- Repo-setup checklist for the harness — `claude-skills/templates/harness-setup.md`
- Harness design (gates, Stop hook, subagents) — `claude-skills/docs/AGENT_HARNESS.md`
- The autonomous loop itself — `claude-skills/skills/elves/README.md`

## What's the "skills library" vs the "harness"?

- **Skills library** = the `eng-skills` Claude Code **plugin** (7 skills:
  `karpathy-guidelines`, `principal-production-engineer`,
  `strategic-engineering-planner`, `implementation-plan`, `cpp-systems-internals`,
  `auto-research`, `elves`). It feeds Claude's reasoning and is routed to
  automatically by task type. Distributed via the `claude-skills` marketplace.
- **Autonomous harness** = the **`elves`** skill — the overnight, multi-batch
  loop (orient → implement → validate → review → judge → document → commit/push)
  that runs unattended against a verification gate and a constitution.

## What is already wired in this repo

| File | Purpose |
|---|---|
| `.claude/settings.json` | Registers the `claude-skills` marketplace and enables `eng-skills@claude-skills`. When the repo folder is **trusted**, Claude Code prompts to auto-install — every teammate/agent who clones gets the skills with no manual commands. |
| `CLAUDE.md` | Always-on routing table: tells Claude which skill to load for which task. Loaded into context every session. |
| `scripts/check.sh` | The verification **gate** (wraps Bazel). The harness runs it after every batch and will not advance on a non-zero exit. |
| `docs/constitution.md` | The **ungameable promises** the elves Judge checks each batch (anti-gaming layer). Review/replace the starter entries. |

## Finish the setup (one-time)

1. **Install the plugin for this session** (the repo wiring auto-prompts on folder
   trust; these two commands do it explicitly / for personal-everywhere use):

   ```text
   /plugin marketplace add Zarand3r/claude-skills
   /plugin install eng-skills@claude-skills
   ```
   Then `/reload-plugins` (or restart). Skills become `/eng-skills:<name>` and
   also auto-invoke by description.

2. **Make the repo pushable** (elves' review loop runs on GitHub PRs). The remote
   `Zarand3r/codon` exists and `gh auth` is green, but there is **no initial
   commit yet** — create one:

   ```bash
   git add -A && git commit -m "Initial import + agent harness setup"
   git push -u origin main
   git push --dry-run    # should now succeed
   ```

3. **Make the gate green.** `scripts/check.sh` wraps `bazel build //... && bazel
   test //...`. The tree currently references headers not present in this slice,
   so a clean build is **not yet green**. Either fix the build, or tell elves to
   stand up a minimal buildable slice + test harness in batch 1. Do not launch an
   unattended run against a red gate.

4. **Fill in the constitution.** Edit `docs/constitution.md` — replace the
   domain-grounded starter promises with the deal-breakers that actually matter
   for the work you're launching.

5. **(Long runs) survive sleep + get notified.** Run Claude inside `tmux` (and
   `caffeinate -dimsu` on macOS) so the session persists; optionally wire a
   Slack/ntfy webhook (see `claude-skills/skills/elves/references/tool-config-examples.md`).

## Pre-launch sanity check

```bash
gh auth status                 # green
git push --dry-run             # push access (needs step 2 first)
./scripts/check.sh             # exits 0 on a clean checkout (needs step 3)
test -f docs/constitution.md   # present
```

When those pass, invoke the harness (`/eng-skills:elves` or just describe an
overnight run), plan the run (~30 min), freeze the plan, then launch and walk
away. Day-to-day attended work needs none of this — the skills route in
automatically via `CLAUDE.md` as soon as the plugin is installed (step 1).
