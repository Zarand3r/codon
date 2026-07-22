/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_LAYOUT_H
#define SLATE_LAYOUT_H

#include "src/bullwinkle/all/B2.h"
#include "src/bullwinkle/all/B2c.h"
#include "src/bullwinkle/all/SlateElement.h"
#include "src/bullwinkle/all/SlatePathMap.h"
#include "src/bullwinkle/all/core/AlignedBuffer.h"
#include "src/bullwinkle/all/slate_accessor.h"
#include "src/bullwinkle/all/slate_elem_access_t.enum.h"
#include "src/bullwinkle/all/slate_info.h"
#include "src/bullwinkle/all/slate_path.h"
#include "src/bullwinkle/all/slate_shard_t.enum.h"
#include "src/bullwinkle/all/slate_subsystem_id_t.enum.h"

#include <array>
#include <map>
#include <string_view>

class SlateLayoutUto;

namespace Drone
{
    /**
     * Records the layout of a future Slate without actually allocating
     * the real storage for it. Used by SlateMemory to construct the final
     * memory map.
     *
     * Users should interact with SlateBuilder instead.
     */
    class SlateLayout
    {
    public:
        SlateLayout();

        bool allocate_element(const std::string_view path,
                              const slate_type_t type_id,
                              const size_t value_size,
                              const size_t alignment,
                              const slate_shard_t shard,
                              const slate_elem_access_t access_policy,
                              const slate_validator_t &validator,
                              const slate_subsystem_id_t subsystem_id,
                              slate_element_t &element_id,
                              void *&mem);

        bool create_view_element(const slate_element_t parent_id,
                                 const slate_type_t parent_type_id,
                                 const size_t element_offset,
                                 const std::string_view path,
                                 const slate_type_t type_id,
                                 const size_t value_size,
                                 const size_t alignment,
                                 const slate_subsystem_id_t subsystem_id,
                                 slate_element_t &view_element_id);

        /**
         * @return A read-only reference to the set of all known Slate elements.
         */
        const SlatePathMap &get_elements() const
        {
            return elements;
        }

        bool get_element(const slate_element_t element_id,
                         const slate_type_t type_id,
                         std::string_view &path,
                         const SlateElementMetadata *&metadata) const;

        bool get_element(const std::string_view path,
                         const SlateElementMetadata *&metadata) const;

        bool get_element_id(const std::string_view path,
                            const slate_type_t type_id,
                            slate_element_t &element_id) const;

        bool get_element_id(const std::string_view path,
                            const slate_type_t type_id,
                            const slate_elem_access_t access_elevation,
                            slate_element_t &element_id) const;

        bool build_element_id(const std::string_view path,
                              const SlateElementMetadata &metadata,
                              const slate_elem_access_t access_elevation,
                              slate_element_t &element_id) const;

        bool
        get_element_access_policy(const slate_element_t element_id,
                                  const slate_type_t type_id,
                                  slate_elem_access_t &access_policy) const;

        bool get_element_type(const std::string_view path,
                              slate_type_t &type_id) const;

        bool get_first_element_path(const slate_element_t element_id,
                                    const slate_type_t type_id,
                                    std::string_view &path) const;

        bool get_element_path(const slate_element_t element_id,
                              std::string_view &path) const;

        bool get_element_initial_memory(const slate_element_t element_id,
                                        const slate_type_t type_id,
                                        B2c &data) const;

        slate_validator_t
        get_element_validator(const slate_element_t element_id,
                              const slate_type_t type_id) const;

        bool path_exists(const std::string_view path) const;
        bool element_exists(const std::string_view path,
                            const slate_type_t type_id) const;
        bool can_write(const slate_element_t element_id) const;
        bool must_validate(const slate_element_t element_id) const;

        bool get_shard_size(const slate_shard_t shard, size_t &size) const;

        /**
         * Get the initial shard contents.
         *
         * @param shard Shard index.
         *
         * @return The initial contents of a shard.
         */
        const AlignedBuffer &get_initial_values(const slate_shard_t shard) const
        {
            FswDebugAssert(shard < num_slate_shard_t);

            return initial_values[shard];
        }

        bool finalize();

    private:
        std::string_view allocate_path(const std::string_view _path);

        friend class ::SlateLayoutUto;

        /**
         * The set of all known Slate elements. This member must be deleted last
         * because it keeps all Slate element paths alive.
         */
        SlatePathMap elements;

        /**
         * Pool for string allocation.
         */
        MonotonicPool string_pool;

        /**
         * The map from a element index to the validator used by the
         * element. There are only few elements with validators, so there is no
         * good reason to keep them in SlateElementMetadata.
         */
        std::map<slate_index_t, slate_validator_t> idx_to_validator {};

        /**
         * Maximum access policy for each shard.
         */
        std::array<slate_elem_access_t, num_slate_shard_t> shard_policy {};

        /**
         * Packed initial element values.
         */
        std::array<AlignedBuffer, num_slate_shard_t> initial_values {};

        /**
         * Shard sizes.
         */
        std::array<slate_offset_t, num_slate_shard_t> shard_size {};

        /**
         * Free list for each shard. This is a map from size -> offset,
         * where each entry points to a free block of that size (and alignment).
         * Blocks are power of 2 sizes always smaller than
         * AlignedBuffer::max_alignment.
         * @{
         */
        using free_chunk_map_t = std::multimap<slate_offset_t, slate_offset_t>;
        std::array<free_chunk_map_t, num_slate_shard_t> shard_free_list {};
        /**
         * @}
         */

        bool add_element_and_build_id(const std::string_view path,
                                      SlateElementMetadata metadata,
                                      const slate_validator_t &validator,
                                      slate_element_t &element_id);
    };

} /* end namespace Drone */

#endif /* SLATE_LAYOUT_H */