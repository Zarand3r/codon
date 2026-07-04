/**
 * @author Richard Bao
 *
 * slate_info — the umbrella header the Slate consumers include for the full
 * element vocabulary: the packed element id (slate_id), the shard/access/
 * permission enums (slate_enums, via slate_id), the stable type id (slate_type),
 * and the fsw assert macros the inline token/accessor code uses.
 *
 * `slate_tokens.h`, `Slate.h`, `SlateBuilder.h`, etc. `#include` this one header.
 *
 * NOTE: the `slate_info<T>` value trait (R/W/O, size/alignment, copy/from_mem)
 * that Slate/SlateBuilder use to move typed values in and out of shard memory is
 * NOT defined yet — it lands with the increment whose consumer (Slate.h) it must
 * satisfy, so that consumer's own compile validates it rather than a mirror test.
 * The token layer (slate_tokens.h) needs only the ids + type id below.
 */

#ifndef SLATE_INFO_H
#define SLATE_INFO_H

#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/slate_id.h"
#include "src/bullwinkle/all/slate_type.h"

#endif /* SLATE_INFO_H */
