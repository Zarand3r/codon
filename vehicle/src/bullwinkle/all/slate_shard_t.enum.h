/**
 * @author Richard Bao
 *
 * Compatibility shim. The imported consumers `#include` per-enum headers named
 * `<enum>.enum.h` (the original tree generated these). We dropped enum codegen
 * (IMPLEMENTATION_PLAN §8 D3, revised): `slate_shard_t` and its `slate_shard_t_sym`
 * reflection live canonically in `slate_enums.h` with a runtime SymbolTable. This
 * shim just re-exports that, so the imported `#include ".../slate_shard_t.enum.h"`
 * resolves without touching the consumers.
 */

#ifndef SLATE_SHARD_T_ENUM_H
#define SLATE_SHARD_T_ENUM_H

#include "src/bullwinkle/all/slate_enums.h"

#endif /* SLATE_SHARD_T_ENUM_H */
