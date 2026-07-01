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
        SymbolTable build_shard_sym()
        {
            SymbolTable t;
            FswAssert(t.add("static", shard_static));
            FswAssert(t.add("sync", shard_sync));
            FswAssert(t.add("nonsync", shard_nonsync));
            FswAssert(t.add("cyclic", shard_cyclic));
            FswAssert(t.add("sync_no_telem", shard_sync_no_telem));
            FswAssert(t.add("nonsync_no_telem", shard_nonsync_no_telem));
            FswAssert(t.add("cyclic_no_telem", shard_cyclic_no_telem));
            return t;
        }

        SymbolTable build_access_sym()
        {
            SymbolTable t;
            FswAssert(t.add("private", slate_private));
            FswAssert(t.add("read_only", slate_read_only));
            FswAssert(t.add("read_write", slate_read_write));
            return t;
        }
    } /* anonymous namespace */

    const SymbolTable slate_shard_t_sym = build_shard_sym();
    const SymbolTable slate_elem_access_t_sym = build_access_sym();

} /* end namespace Drone */
