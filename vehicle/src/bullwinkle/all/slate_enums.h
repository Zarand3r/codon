/**
 * @author Richard Bao
 *
 * Slate shard set, element-access policy, and permission algebra.
 *
 * These three enums are the fixed vocabulary the Slate is built from. They are
 * declared-and-used but never defined across the imported tree; the contract
 * here is inferred from the call sites (SlateLayout, SlateBuilder, FtRuntime,
 * SlateMemory) and pinned to SYSTEM_DESIGN.md §Shards / §Purpose-and-redundancy.
 *
 * Cold path: shards/access/permissions are resolved at Slate *build* time. The
 * only runtime use is `shard` as an array index (SlateMemory::shard_table) and
 * a packed field of the element ID (slate_info.h); those are plain integer ops.
 *
 * Included by slate_info.h (which needs `slate_shard_t` for the packed ID).
 */

#ifndef SLATE_ENUMS_H
#define SLATE_ENUMS_H

#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/enum/SymbolTable.h"

namespace Drone
{
    /**
     * The seven Slate memory shards. The value doubles as an index into the
     * per-shard tables (SlateMemory::shard_table, SlateLayout::shard_size), so
     * the enumerators are dense and start at zero. See SYSTEM_DESIGN.md §Shards.
     *
     * The `_no_telem` variants mirror their base shard but are excluded from
     * telemetry framing; the sharing/voting policy is otherwise identical.
     */
    enum slate_shard_t
    {
        shard_static = 0,          /**< immutable after build (constants).      */
        shard_sync = 1,            /**< replicated + voted across strings.      */
        shard_nonsync = 2,         /**< string-private, persists across frames. */
        shard_cyclic = 3,          /**< string-private, reset every frame.      */
        shard_sync_no_telem = 4,   /**< sync, not telemetered.                  */
        shard_nonsync_no_telem = 5,/**< nonsync, not telemetered.               */
        shard_cyclic_no_telem = 6, /**< cyclic, not telemetered.                */

        num_slate_shard_t = 7,     /**< count of real shards; also the sentinel */
        shard_invalid = num_slate_shard_t /**< "no shard" (>= num, so `shard <
                                            *   num_slate_shard_t` rejects it).  */
    };

    /**
     * How creation into a shard is gated. This is the single source of truth
     * for the creation-time permission boundary (SYSTEM_DESIGN.md): it is
     * classified by an exhaustive switch below, so adding a shard fails to
     * compile (`-Wswitch -Werror`) until its create policy is stated here.
     */
    enum slate_shard_create_class_t
    {
        slate_create_class_open,    /**< static/cyclic: create right suffices.  */
        slate_create_class_sync,    /**< replicated+voted: needs c_sync.         */
        slate_create_class_nonsync  /**< string-private: needs c_nonsync.        */
    };

    inline slate_shard_create_class_t
    slate_shard_create_class(const slate_shard_t shard)
    {
        switch (shard)
        {
        case shard_sync:
        case shard_sync_no_telem:
            /* Only the control string may create voted state. */
            return slate_create_class_sync;
        case shard_nonsync:
        case shard_nonsync_no_telem:
            /* The control string is barred from string-private state. */
            return slate_create_class_nonsync;
        case shard_static:
        case shard_cyclic:
        case shard_cyclic_no_telem:
            /* Both control and runtime keep per-frame cyclic scratch (e.g.
             * BasicControl's reset_counters / autosequence flags), and static
             * constants are seeded by the permission-exempt super builder. */
            return slate_create_class_open;
        case num_slate_shard_t: /* == shard_invalid; not a real shard */
            break;
        }
        return slate_create_class_open;
    }

    /**
     * True for the shards whose contents are replicated and voted across
     * strings. Defined in terms of the classifier so there is one source of
     * truth for "sync-ness" (voting/telemetry will reuse this).
     */
    inline bool slate_shard_is_sync_class(const slate_shard_t shard)
    {
        return slate_shard_create_class(shard) == slate_create_class_sync;
    }

    /**
     * Element access policy. Ordered from least to most permissive so
     * SlateLayout can compare with `>` (reject a request above a shard's
     * policy) and combine with std::max (an element takes the more permissive
     * of its declared policy and any elevation).
     */
    enum slate_elem_access_t
    {
        slate_private = 0,    /**< not externally addressable.       */
        slate_read_only = 1,  /**< readable, never writable.         */
        slate_read_write = 2  /**< readable and writable.            */
    };

    /**
     * Slate permission bitmask. A permission is the set of rights a Slate
     * handle grants; sub-slates may only narrow it (never elevate). The
     * create-sync / create-nonsync qualifier bits are what separate the control
     * string (may create voted `sync` state) from every other subsystem.
     *
     * The individual bits back the predicates below; callers use the named
     * composite constants (`slate_permission_r/rc/rwc`) and the qualifier masks
     * (`slate_permission_c_sync/c_nonsync`) with slate_permission_deny().
     */
    enum slate_permission_t : uint
    {
        slate_permission_none = 0,

        slate_permission_bit_read = 1u << 0,
        slate_permission_bit_write = 1u << 1,
        slate_permission_bit_create = 1u << 2,
        slate_permission_bit_create_sync = 1u << 3,
        slate_permission_bit_create_nonsync = 1u << 4,

        /** Read-only. */
        slate_permission_r = slate_permission_bit_read,

        /** Read + create (either class), no write. */
        slate_permission_rc = slate_permission_bit_read |
                              slate_permission_bit_create |
                              slate_permission_bit_create_sync |
                              slate_permission_bit_create_nonsync,

        /** Read + write + create (either class): the unrestricted root. */
        slate_permission_rwc = slate_permission_bit_read |
                              slate_permission_bit_write |
                              slate_permission_bit_create |
                              slate_permission_bit_create_sync |
                              slate_permission_bit_create_nonsync,

        /** Qualifier masks — used only as the second arg to _deny(). */
        slate_permission_c_sync = slate_permission_bit_create_sync,
        slate_permission_c_nonsync = slate_permission_bit_create_nonsync
    };

    /**
     * Remove `mask`'s rights from `base` (e.g. deny sync-create).
     */
    inline slate_permission_t slate_permission_deny(const slate_permission_t base,
                                                    const slate_permission_t mask)
    {
        return static_cast<slate_permission_t>(base & ~static_cast<uint>(mask));
    }

    /**
     * True if `parent` grants every right `child` does (child ⊆ parent). Used
     * to forbid privilege elevation when opening a sub-slate.
     */
    inline bool slate_permission_is_superset(const slate_permission_t parent,
                                             const slate_permission_t child)
    {
        return (static_cast<uint>(parent) & static_cast<uint>(child)) ==
               static_cast<uint>(child);
    }

    /**
     * The rights common to both — a sub-slate's effective permission is the
     * intersection of its parent's and what it requested.
     */
    inline slate_permission_t
    slate_permission_take_subset(const slate_permission_t parent,
                                 const slate_permission_t requested)
    {
        return static_cast<slate_permission_t>(static_cast<uint>(parent) &
                                               static_cast<uint>(requested));
    }

    inline bool slate_can_read(const slate_permission_t p)
    {
        return (p & slate_permission_bit_read) != 0;
    }

    inline bool slate_can_write(const slate_permission_t p)
    {
        return (p & slate_permission_bit_write) != 0;
    }

    /**
     * True if permission `p` may create an element in `shard`. The general
     * create right is always required; the shard's create-class then decides
     * which qualifier (if any) it additionally needs. This is what lets the
     * control string (create right + sync qualifier only) still create cyclic
     * scratch while being barred from string-private `nonsync` state.
     */
    inline bool slate_can_create(const slate_permission_t p,
                                 const slate_shard_t shard)
    {
        if ((p & slate_permission_bit_create) == 0)
        {
            return false;
        }
        switch (slate_shard_create_class(shard))
        {
        case slate_create_class_sync:
            return (p & slate_permission_bit_create_sync) != 0;
        case slate_create_class_nonsync:
            return (p & slate_permission_bit_create_nonsync) != 0;
        case slate_create_class_open:
            return true;
        }
        return false;
    }

    /**
     * Reflection tables (value -> name) for diagnostics and config parsing.
     * Populated once at static-init in slate_enums.cc.
     */
    extern const SymbolTable slate_shard_t_sym;
    extern const SymbolTable slate_elem_access_t_sym;

} /* end namespace Drone */

#endif /* SLATE_ENUMS_H */
