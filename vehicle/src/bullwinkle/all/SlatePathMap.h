/**
 * @author Richard Bao
 *
 * SlatePathMap — the element directory: element path -> its layout metadata, plus a
 * stable, dense, **1-based** index bijection.
 *
 * The keys are `std::string_view`s that point into the layout's interned path pool
 * (SlateLayout owns the backing storage; this map does not copy the characters). The
 * 1-based index is what gets packed into the element's `slate_element_t` — and it
 * starts at 1 on purpose: index 0 is reserved so a bound id is never the default (0)
 * sentinel (see slate_id.h `is_valid`). `slate_id_buildup` also rejects index 0, so
 * this is the sole producer that must honour the invariant, and it does.
 */

#ifndef SLATE_PATH_MAP_H
#define SLATE_PATH_MAP_H

#include "src/bullwinkle/all/SlateElement.h"
#include "src/bullwinkle/all/slate_id.h"

#include <map>
#include <string_view>
#include <utility>
#include <vector>

namespace Drone
{
    class SlatePathMap
    {
    public:
        typedef std::map<std::string_view, SlateElementMetadata> map_t;
        typedef map_t::const_iterator const_iterator;

        const_iterator find(const std::string_view path) const
        {
            return by_path.find(path);
        }
        const_iterator begin() const { return by_path.begin(); }
        const_iterator end() const { return by_path.end(); }
        size_t size() const { return by_path.size(); }
        bool empty() const { return by_path.empty(); }

        /**
         * Register path -> metadata. On a fresh path the element is assigned the
         * next dense 1-based index. Returns {iterator, inserted}: inserted is false
         * (and no index is consumed) if the path already exists.
         */
        std::pair<const_iterator, bool>
        insert(std::pair<std::string_view, SlateElementMetadata> kv)
        {
            const std::pair<map_t::iterator, bool> res = by_path.insert(std::move(kv));
            if (res.second)
            {
                /* 1-based, dense, monotonic: honours the slate_id index>=1 invariant. */
                by_index.push_back(res.first);
                ptr_to_index[&res.first->second] =
                    static_cast<slate_index_t>(by_index.size());
            }
            return std::pair<const_iterator, bool>(res.first, res.second);
        }

        /** The 1-based index of the element `it` refers to (0 if `it` is unknown). */
        slate_index_t iterator_to_id(const_iterator it) const
        {
            if (it == by_path.end())
            {
                return 0; // never dereference end()
            }
            const auto found = ptr_to_index.find(&it->second);
            return found == ptr_to_index.end() ? 0 : found->second;
        }

        /** The element for a 1-based index, or end() if out of range. */
        const_iterator id_to_iterator(const slate_index_t idx) const
        {
            if (idx < 1 || idx > by_index.size())
            {
                return by_path.end();
            }
            return by_index[idx - 1];
        }

        void clear()
        {
            by_path.clear();
            by_index.clear();
            ptr_to_index.clear();
        }

    private:
        map_t by_path{};
        /* by_index[idx-1] -> iterator (dense id -> element). map nodes are address-
         * stable, so storing iterators / node addresses across inserts is safe. */
        std::vector<const_iterator> by_index{};
        std::map<const SlateElementMetadata *, slate_index_t> ptr_to_index{};
    };

} /* end namespace Drone */

#endif /* SLATE_PATH_MAP_H */
