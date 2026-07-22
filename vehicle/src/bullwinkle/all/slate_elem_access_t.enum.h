/**
 * @author Richard Bao
 *
 * Compatibility shim (see slate_shard_t.enum.h). `slate_elem_access_t` and its
 * `slate_elem_access_t_sym` reflection live canonically in `slate_enums.h`; this
 * re-exports them so the imported `#include ".../slate_elem_access_t.enum.h"` resolves.
 */

#ifndef SLATE_ELEM_ACCESS_T_ENUM_H
#define SLATE_ELEM_ACCESS_T_ENUM_H

#include "src/bullwinkle/all/slate_enums.h"

#endif /* SLATE_ELEM_ACCESS_T_ENUM_H */
