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
         * Names carry the "shard_" prefix: SlateLayout formats them as
         * `get(shard).substr(strlen("shard_"))`, so a shorter name would throw
         * std::out_of_range and the wrong prefix length would mangle the rest.
         */
        SymbolTable build_shard_sym()
        {
            SymbolTable t;
            FswAssert(t.add("shard_static", shard_static));
            FswAssert(t.add("shard_sync", shard_sync));
            FswAssert(t.add("shard_nonsync", shard_nonsync));
            FswAssert(t.add("shard_cyclic", shard_cyclic));
            FswAssert(t.add("shard_sync_no_telem", shard_sync_no_telem));
            FswAssert(t.add("shard_nonsync_no_telem", shard_nonsync_no_telem));
            FswAssert(t.add("shard_cyclic_no_telem", shard_cyclic_no_telem));
            return t;
        }

        /*
         * Names carry the "slate_" prefix for the same reason: SlateLayout
         * formats them as `get(access).substr(strlen("slate_"))`.
         */
        SymbolTable build_access_sym()
        {
            SymbolTable t;
            FswAssert(t.add("slate_private", slate_private));
            FswAssert(t.add("slate_read_only", slate_read_only));
            FswAssert(t.add("slate_read_write", slate_read_write));
            return t;
        }
    } /* anonymous namespace */

    const SymbolTable slate_shard_t_sym = build_shard_sym();
    const SymbolTable slate_elem_access_t_sym = build_access_sym();

} /* end namespace Drone */
