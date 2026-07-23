# Imported-code audit (tree-wide, per engineering-lessons.md)

Result of applying the mechanical lessons (guard audit, truncation audit, typo
sweep, first-error catalog) to the **entire** tree — not just the gated Slate
foundation. Refresh this whenever a new dependency layer lands (each newly
compiling consumer can expose defects hidden behind its missing includes).

**Last run: 2026-07-22 · branch `treewide-audit` · gate GREEN**

## Audit results

| Audit | Method | Findings |
|---|---|---|
| **Include-guard collisions** (the `Slate.h`/`SLATE_BUILDER_H` bug class) | every `#ifndef` guard name, deduped across files | **0** — the Slate.h collision was the only one |
| **Guard balance / missing guards** | `#if`/`#endif` counts; guard-or-pragma presence | **0** unbalanced, 0 unguarded |
| **Brace-balance truncations** | `{`/`}` outside comments+strings | **0** (SlateCombiner.h flagged raw but balances once doc-comment code examples are excluded) |
| **Misspelling sweep** (curated list incl. the observed family: teh/overship/pointes/retreive/attemptse/…) | word-boundary grep, all `.h`/`.cc` | **1** (`supress` in `io/StreamConnection.cc`) — fixed |
| **First-error catalog** (below) | `-fsyntax-only` each ungated `.cc` | **all DEP-class; zero DEFECT-class** |

## Ungated `.cc` readiness map (what each is waiting for)

Every not-yet-gated translation unit fails on a *missing dependency*, not a
defect. This doubles as the P2+ work map — the blocking header tells you which
phase unlocks it:

| TU | First missing dep | Unlocked by |
|---|---|---|
| `FdBag.cc`, `Server.cc`, `io/TcpServer.cc` | `FdEventSink.h` | **P2** event loop |
| `io/{TcpConnection,TcpMultiServerConnection,UdpConnection}.cc` | `EventList.h` | **P2** |
| `io/FdDgramChannel.cc` | `BipBuffer.h` | **P2** |
| `io/FdStreamChannel.cc` | `DataQueue.h` | **P2** |
| `io/StreamConnection.cc` | `io/StreamChannel.h` | **P2** |
| `async/DeferredCallbackQueue.cc` | `EventLoop.h` | **P2** |
| `SlateCombiner.cc` | `Configs.h` | **P3** config (last P1 piece blocked by a P3 leaf — pull `Configs` forward or stub) |
| `StateMachine.cc` | `CommandTable.h` | **P6** commands |
| `curl/CurlFileDownload.cc` | `hsm/SslPrivateKeyMethod.h` | **P6** security |
| `flight/common/{BasicControl,FtRuntime}.cc` | `AdcPowersaver.h` / `AdcScaler.h` | **P7/P9** |
| `flight/sat/.../DroneGnc{Control,Runtime}.cc` | `Configs.h` / `AlertBufferManager.h` | **P8/P10** |

## Standing rule (from engineering-lessons.md §5)

Defects behind missing deps are invisible to this audit. As each dependency
lands, the newly-compiling consumer joins the gate **immediately** (relaxed tier
for imported diagnostics), and its first real compile is expected to surface the
next batch of truncation/typo defects — budget for that in every P2+ increment,
and re-run this audit's checks at each phase boundary.

Notable planning fact surfaced: **`SlateCombiner.cc` (the last P1 piece) blocks
on `Configs.h`, a P3 leaf** — either pull a minimal `Configs` forward or stub the
config dependency to close P1 before P2.
