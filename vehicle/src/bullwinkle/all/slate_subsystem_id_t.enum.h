/**
 * @author Richard Bao
 *
 * Compatibility shim (see slate_shard_t.enum.h). Unlike shard/access, the subsystem
 * id has no reflection table in present usage — it is a plain 32-bit id identifying
 * the subsystem that created an element (build-time bookkeeping). This is its
 * canonical home; SlateElement.h includes it.
 */

#ifndef SLATE_SUBSYSTEM_ID_T_ENUM_H
#define SLATE_SUBSYSTEM_ID_T_ENUM_H

#include "src/bullwinkle/all/core/drone_types.h"

namespace Drone
{
    /** Identifies the subsystem that owns/created an element. */
    typedef UINT32 slate_subsystem_id_t;
}

#endif /* SLATE_SUBSYSTEM_ID_T_ENUM_H */
