/**
 * @author Richard Bao
 *
 * Static reflection tables for the Slate enums. See slate_enums.h.
 */

#include "src/bullwinkle/all/slate_enums.h"

#include "src/bullwinkle/all/core/fsw.h"

namespace Drone
{
    namespace
    {
        /*
         * Register an enumerator under a name *stringized from the enumerator
         * token itself*. Because the enumerators are literally named
         * `shard_<x>` / `slate_<x>`, the reflection name always carries the
         * exact "shard_" / "slate_" prefix that SlateLayout strips with
         * `get(...).substr(strlen("shard_"|"slate_"))` — the name cannot drift
         * from the symbol, which is the whole point (a hand-typed name is how
         * the prefix bug happened in the first place).
         */
#define SLATE_SYM_ADD(table, enumerator) \
    FswAssert((table).add(#enumerator, (enumerator)))

        SymbolTable build_shard_sym()
        {
            SymbolTable t;
            SLATE_SYM_ADD(t, shard_static);
            SLATE_SYM_ADD(t, shard_sync);
            SLATE_SYM_ADD(t, shard_nonsync);
            SLATE_SYM_ADD(t, shard_cyclic);
            SLATE_SYM_ADD(t, shard_sync_no_telem);
            SLATE_SYM_ADD(t, shard_nonsync_no_telem);
            SLATE_SYM_ADD(t, shard_cyclic_no_telem);
            return t;
        }

        SymbolTable build_access_sym()
        {
            SymbolTable t;
            SLATE_SYM_ADD(t, slate_private);
            SLATE_SYM_ADD(t, slate_read_only);
            SLATE_SYM_ADD(t, slate_read_write);
            return t;
        }

#undef SLATE_SYM_ADD
    } /* anonymous namespace */

    const SymbolTable slate_shard_t_sym = build_shard_sym();
    const SymbolTable slate_elem_access_t_sym = build_access_sym();

} /* end namespace Drone */
