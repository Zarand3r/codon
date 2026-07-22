// Compile witness for the .enum.h compatibility shims: they must resolve their
// symbols and coexist with SlateElement.h / SlatePathMap.h (which also pull
// slate_subsystem_id_t.enum.h) without a double definition. Object-only gate.
#include "src/bullwinkle/all/SlateElement.h"
#include "src/bullwinkle/all/SlatePathMap.h"
#include "src/bullwinkle/all/slate_elem_access_t.enum.h"
#include "src/bullwinkle/all/slate_shard_t.enum.h"
#include "src/bullwinkle/all/slate_subsystem_id_t.enum.h"

using namespace Drone;

// Odr-use symbols from each shim so a missing/renamed one fails to compile.
void slate_enum_headers_witness()
{
    const slate_shard_t s = shard_sync;                 // slate_shard_t.enum.h
    const slate_elem_access_t a = slate_read_write;     // slate_elem_access_t.enum.h
    const slate_subsystem_id_t id = 0;                  // slate_subsystem_id_t.enum.h
    (void)slate_shard_t_sym.get(s);
    (void)slate_elem_access_t_sym.get(a);
    (void)id;
    (void)num_slate_shard_t;
}
