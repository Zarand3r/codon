#pragma once

// Hotsync-analyzer annotations. A separate tool reads these to enforce the
// "no private state outside the Slate / no runtime allocation in RUNTIME methods"
// discipline. In a normal build they are no-ops.
//   RUNTIME            — marks a hot-path method/qualifier (must be allocation-free)
//   INFRASTRUCTURE(T)  — marks an init-time member of type T (identity wrapper)
//   HOTSYNC_EXEMPT     — opts a definition out of the analyzer

#ifndef RUNTIME
#define RUNTIME
#endif

#ifndef INFRASTRUCTURE
#define INFRASTRUCTURE(...) __VA_ARGS__
#endif

#ifndef HOTSYNC_EXEMPT
#define HOTSYNC_EXEMPT
#endif
