/**
 * @author Richard Bao
 *
 * slate_type — a deterministic, cross-string-stable identifier for a C++ type.
 *
 * Every Slate element is tagged with the id of its value type so a load/store,
 * a command-by-name, and a cross-string layout comparison can all check they
 * are talking about the same type. The id must be **identical on all three
 * strings** (they compare/serialize type ids while agreeing on layout), which
 * rules out `typeid(T).hash_code()` (unspecified, not stable across runs).
 *
 * Scheme (decided with the human): hash the compiler's per-type function-name
 * string (`__PRETTY_FUNCTION__`, which embeds T's spelling) with the repo's
 * `digest_xxh128`, folded to 64 bits. The three strings run the *same* binary,
 * so the name string — and therefore the id — is identical across them; distinct
 * types produce distinct strings and (at 64 bits) distinct ids.
 *
 * Cold path: called at Slate *build* time to tag/bind elements, never in a
 * RUNTIME hot method. The result is memoized per type.
 */

#ifndef SLATE_TYPE_H
#define SLATE_TYPE_H

#include "src/bullwinkle/all/core/drone_types.h"
#include "src/hash/xxh.h"

#include <cstring>

namespace Drone
{
    /** 64-bit type identifier (matches SlateLayout's FswAbortIfEqUint64 usage). */
    typedef UINT64 slate_type_t;

    /** The "no type" sentinel; a real type id is never equal to this. */
    static const slate_type_t slate_type_invalid = 0;

    namespace slate_type_detail
    {
        /*
         * A per-type name string. __PRETTY_FUNCTION__ expands to a signature that
         * contains T's spelling, so it is distinct per T and constant for a fixed
         * binary. It is compiler-specific — which is fine: the three strings run
         * the identical binary, so they see the identical string.
         */
        template <typename T>
        inline const char *type_name()
        {
#if defined(__GNUC__) || defined(__clang__)
            return __PRETTY_FUNCTION__;
#else
            return __func__; /* last-resort; distinctness weakens off gcc/clang */
#endif
        }

        /* Fixed, non-zero seed so the id stream is stable and reproducible. */
        inline Hash128 type_seed()
        {
            return Hash128(0x9E3779B97F4A7C15ULL, 0xF39CC0605CEDC834ULL);
        }
    } /* namespace slate_type_detail */

    /**
     * The stable id for type T.
     *
     * @return A non-zero slate_type_t, identical across strings running the same
     *         binary and distinct per type.
     */
    template <typename T>
    inline slate_type_t slate_type_id()
    {
        /* Memoize: the hash is cold but pointless to recompute per call. */
        static const slate_type_t id = []() {
            const char *name = slate_type_detail::type_name<T>();
            const Hash128 h =
                digest_xxh128(name, std::strlen(name), slate_type_detail::type_seed());
            const slate_type_t folded = h.u64[0] ^ h.u64[1];
            /* Never collide with the invalid sentinel. */
            return folded == slate_type_invalid ? ~static_cast<slate_type_t>(0)
                                                : folded;
        }();
        return id;
    }

} /* end namespace Drone */

#endif /* SLATE_TYPE_H */
