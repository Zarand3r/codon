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
#include <map>
#include <string>

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
     * Reflection record for a slate type: its id and human-readable name. Returned
     * (by pointer, no allocation) from get_type_info for diagnostics — e.g. a
     * type-mismatch error naming the expected vs. actual type.
     */
    struct slate_type_info_t
    {
        slate_type_t id;
        std::string name;
    };

    namespace slate_type_detail
    {
        /**
         * Global id -> info registry. Populated once per type by slate_type_id<T>()
         * on first use. A Meyers singleton (function-local static) avoids static-init
         * order issues. Cold path only (build-time type tagging), so a std::map is
         * fine. The registry never shrinks; entries are stable (pointer returned to
         * callers), so std::map's node stability is required.
         */
        inline std::map<slate_type_t, slate_type_info_t> &type_registry()
        {
            static std::map<slate_type_t, slate_type_info_t> registry;
            return registry;
        }

        inline slate_type_t register_type(const slate_type_t id, const char *name)
        {
            auto &reg = type_registry();
            /* insert-if-absent; the stored name is a pointer to the static
             * __PRETTY_FUNCTION__ string, which lives for the whole program. */
            reg.emplace(id, slate_type_info_t{id, std::string(name)});
            return id;
        }
    } /* namespace slate_type_detail */

    /**
     * The stable id for type T.
     *
     * @return A non-zero slate_type_t, identical across strings running the same
     *         binary and distinct per type. Registers T's {id, name} on first use so
     *         get_type_info can later recover the name for diagnostics.
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
            const slate_type_t final_id =
                folded == slate_type_invalid ? ~static_cast<slate_type_t>(0) : folded;
            return slate_type_detail::register_type(final_id, name);
        }();
        return id;
    }

    /**
     * Type reflection lookup — maps a type id back to its {id, name}. Only ids that
     * have been minted by slate_type_id<T>() are known (which, by the time a
     * type-mismatch check runs, both the expected and actual types are). Returns a
     * pointer into the stable registry; do not free.
     */
    namespace slate_type_info_utils
    {
        inline bool get_type_info(const slate_type_t type,
                                  const slate_type_info_t *&info)
        {
            auto &reg = slate_type_detail::type_registry();
            const auto found = reg.find(type);
            if (found == reg.end())
            {
                return false;
            }
            info = &found->second;
            return true;
        }
    } /* namespace slate_type_info_utils */

} /* end namespace Drone */

#endif /* SLATE_TYPE_H */
