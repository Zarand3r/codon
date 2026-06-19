/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#include "src/bullwinkle/all/SlateLayout.h"

#include "src/bullwinkle/all/core/sac.h"

#include <limits>

namespace Drone
{
    /**
     * Constructor.
     */
    SlateLayout::SlateLayout()
    {
        /*
         * Static elements must never be written to. Every other shard is
         * writable by default.
         */
        shard_policy.fill(slate_read_write);
        shard_policy[shard_static] = slate_read_only;
    }

    /**
     * Allocate a new Slate element. The caller is responsible for the
     * initialization of the memory.
     *
     * @param path Path of the new element. May not be a child or the
     *        parent of another element.
     * @param type_id Type ID of the element.
     * @param value_size Memory required to store the element.
     * @param alignment Alignment required to store the element.
     * @param shard Physical shard the element should live in.
     * @param access_policy Element access policy for other users.
     * @param validator The on-write validator to use.
     * @param subsystem_id The subsystem ID that will be assigned to the new
     *        element.
     * @param[out] element_id Returns the element ID.
     * @param[out] mem Returns the pointer to the allocated memory. NULL on
     *             error.
     *
     * @return True on success.
     */
    bool SlateLayout::allocate_element(const std::string_view path,
                                       const slate_type_t type_id,
                                       const size_t value_size,
                                       const size_t alignment,
                                       const slate_shard_t shard,
                                       const slate_elem_access_t access_policy,
                                       const slate_validator_t &validator,
                                       const slate_subsystem_id_t subsystem_id,
                                       slate_element_t &element_id,
                                       void *&mem)
    {
        element_id = slate_element_default;
        mem = NULL;

        SacAbortIfEqUint64(value_size, 0U, false);

        /*
         * Alignment must be positive and a power of 2.
         */
        SacAbortIfNot((alignment != 0) &&
                          ((alignment & -alignment) == alignment),
                      false);
        SacAbortIf(alignment > AlignedBuffer::max_alignment, false);
        SacAbortIf(alignment > value_size, false);

        SacAbortIfNot(shard < num_slate_shard_t, false);

        /*
         * Check that the access policy does not exceed the maximum for
         * that shard.
         */
        if (access_policy > shard_policy[shard])
        {
            const std::string policy_name_long =
                slate_elem_access_t_sym.get(access_policy);
            const std::string policy_name =
                policy_name_long.substr(strlen("slate_"));

            const std::string max_policy_name_long =
                slate_elem_access_t_sym.get(shard_policy[shard]);
            const std::string max_policy_name =
                max_policy_name_long.substr(strlen("slate_"));

            const std::string shard_name =
                slate_shard_t_sym.get(shard).substr(strlen("shard_"));

            SacPrefix();
            dbnprintf(500,
                      ": Access policy '%s' exceeds the maximum policy "
                      "level of '%s' for shard '%s'. Consult "
                      "slate_elem_access_t for access descriptions.\n",
                      policy_name.c_str(),
                      max_policy_name.c_str(),
                      shard_name.c_str());
            return false;
        }

        size_t value_offset = 0;
        size_t free_offset = 0;
        size_t free_size = 0;

        /*
         * Try to find a free chunk.
         */
        free_chunk_map_t &free_list = shard_free_list[shard];
        free_chunk_map_t::iterator it = free_list.lower_bound(value_size);
        if (it != free_list.end())
        {
            value_offset = it->second;
            free_size = it->first - value_size;
            free_offset = it->second + value_size;
            free_list.erase(it);
        }
        else
        {
            /*
             * Calculate the aligned value offset and make sure it does not
             * overflow.
             */
            constexpr auto max_offset =
                std::numeric_limits<slate_offset_t>::max();
            static_assert(AlignedBuffer::max_alignment + 1 <= max_offset);

            SacAbortIfOpUint64(shard_size[shard],
                               >,
                               max_offset - AlignedBuffer::max_alignment + 1,
                               false);
            value_offset = (shard_size[shard] + alignment - 1) & -alignment;
            SacDebugAssert(value_offset >= shard_size[shard]);
            free_offset = shard_size[shard];
            free_size = value_offset - free_offset;

            /*
             * Grow the shard and make sure it does not overflow.
             */
#if __SIZEOF_SIZE_T__ > 4
            SacAbortIfOpUint64(value_size, >, max_offset, false);
#endif
            shard_size[shard] =
                static_cast<slate_offset_t>(value_offset + value_size);
            SacAbortIfOpUint64(value_offset, >=, shard_size[shard], false);
            SacAbortIfNot(initial_values[shard].ensure(shard_size[shard],
                                                       alignment),
                          false);
        }

        /*
         * We have free space.  Carve it into power of two chunks and insert it
         * into the free list. To preserve alignment, we require that the
         * alignment of each chunk equals the size of the chunk.
         */
        if (free_size)
        {
            SacAbortIfNot(free_size < AlignedBuffer::max_alignment, false);
            slate_offset_t size = 1;
            while (size <= free_size)
            {
                if (free_offset & size)
                {
                    free_list.insert(std::make_pair(size, free_offset));
                    free_offset += size;
                    free_size -= size;
                }
                size <<= 1;
            }

            /*
             * Double check we've consumed all the free space.
             *
             * Because the end of the free space is aligned to a power of two,
             * we should always be able to fully utilize the free space by
             * counting up in powers of two (moving from "less aligned" to "more
             * aligned" offsets).
             */
            SacAbortIfNeqUint64(free_size, 0, false);
        }

        /*
         * Create and register the element.
         */
        SacAbortIfNot(
            add_element_and_build_id(
                path,
                SlateElementMetadata(type_id,
                                     shard,
                                     static_cast<slate_offset_t>(value_offset),
                                     static_cast<slate_offset_t>(value_size),
                                     access_policy,
                                     subsystem_id,
                                     false /* is_view_element */),
                validator,
                element_id),
            false);

        /*
         * Success. Return the pointer to the newly allocated memory.
         */
        mem = initial_values[shard].data() + value_offset;
        return true;
    }

    /**
     * Create a new Slate view element.
     *
     * @param parent_id Element ID of the parent.
     * @param parent_type_id Type id of the parent element.
     * @param element_offset The offset inside the parent element to create this
     *                       view element. The data at this offset should match
     *                       type_id or memory can be corrupted.
     * @param path Path of the view element. May not be a child or the
     *             parent of another element.
     * @param type_id Type ID of the view element.
     * @param value_size Size of the view element.
     * @param alignment Alignment of the view element.
     * @param subsystem_id The subsystem ID that will be assigned to the new
     *        element.
     * @param[out] view_element_id Returns the view element ID.
     *
     * @return True on success.
     */
    bool
    SlateLayout::create_view_element(const slate_element_t parent_id,
                                     const slate_type_t parent_type_id,
                                     const size_t element_offset,
                                     const std::string_view path,
                                     const slate_type_t type_id,
                                     const size_t value_size,
                                     const size_t alignment,
                                     const slate_subsystem_id_t subsystem_id,
                                     slate_element_t &view_element_id)
    {
        SacAbortIfEqUint64(value_size, 0U, false);

        /*
         * Alignment must be positive and a power of 2.
         */
        SacAbortIfNot((alignment != 0) &&
                          ((alignment & -alignment) == alignment),
                      false);
        SacAbortIf(alignment > AlignedBuffer::max_alignment, false);

        /*
         * Look up parent element.
         */
        std::string_view parent_path;
        const SlateElementMetadata *parent_metadata;
        SacAbortIfNot(get_element(parent_id,
                                  parent_type_id,
                                  parent_path,
                                  parent_metadata),
                      false);

        SacMsgAbortIf(
            idx_to_validator.count(slate_id_index(parent_id)) &&
                parent_metadata->access_policy > slate_read_only,
            false,
            500,
            "Cannot create slate view element '%.*s' from parent element "
            "'%.*s', because the view would inherit the parent's access policy "
            "allowing writes, and the parent has a write validator.",
            int(path.size()),
            path.data(),
            int(parent_path.size()),
            parent_path.data());

        SacAbortIfOpUint64(element_offset,
                           >
                           , parent_metadata->value_size, false);
        SacAbortIfOpUint64(value_size, >, parent_metadata->value_size, false);
        SacAbortIfOpUint64(element_offset + value_size,
                           >
                           , parent_metadata->value_size, false);

        /*
         * Verify alignment is compatible with parent's alignment.
         */
        SacMsgAbortIf(
            (parent_metadata->value_offset + element_offset) & (alignment - 1),
            false,
            500,
            "Cannot create slate view element '%.*s' of parent element '%.*s'. "
            "The view element's alignment '%zu' and offset '%zu' is not "
            "compatible with the parent's shard offset '%u'.",
            int(path.size()),
            path.data(),
            int(parent_path.size()),
            parent_path.data(),
            alignment,
            element_offset,
            parent_metadata->value_offset);

        /*
         * Create and register the element.
         *
         * Copy the access policy the parent. Note that the parent cannot have a
         * validator, which is checked above.
         */
        SacAbortIfNot(
            add_element_and_build_id(
                path,
                SlateElementMetadata(type_id,
                                     parent_metadata->shard,
                                     static_cast<slate_offset_t>(
                                         parent_metadata->value_offset +
                                         element_offset),
                                     static_cast<slate_offset_t>(value_size),
                                     parent_metadata->access_policy,
                                     subsystem_id,
                                     true /* is_view_element */),
                slate_validator_t {},
                view_element_id),
            false);

        /*
         * Copy the parent's access.
         */
        if (!slate_id_can_write(parent_id))
        {
            view_element_id = slate_id_ro(view_element_id);
        }

        return true;
    }

    /**
     * Retrieve an element by the Slate element ID and type.
     *
     * @param element_id The Slate element ID.
     * @param type_id The element type.
     * @param[out] path Receives the element path.
     * @param[out] metadata Receives a pointer to the element metadata.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element(const slate_element_t element_id,
                                  const slate_type_t type_id,
                                  std::string_view &path,
                                  const SlateElementMetadata *&metadata) const
    {
        SacAbortIfNot(slate_id_is_valid(element_id), false);
        SacAbortIfEqUint64(type_id, slate_type_invalid, false);

        const slate_index_t idx = slate_id_index(element_id);
        const SlatePathMap::const_iterator it = elements.id_to_iterator(idx);
        SacAbortIf(it == elements.end(), false);
        SacMsgAbortIfNot(it->second.type_id == type_id,
                         false,
                         200,
                         "Could not find Slate element ID 0x%016llx with type "
                         "%hu (actual type %hu).",
                         element_id,
                         type_id,
                         it->second.type_id);
        path = it->first;
        metadata = &it->second;
        return true;
    }

    /**
     * Retrieve information associated with a gven element.
     *
     * @param path The Slate element path.
     * @param[out] metadata Receives a pointer to the element metadata.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element(const std::string_view path,
                                  const SlateElementMetadata *&metadata) const
    {
        const auto i = elements.find(path);

        SacMsgAbortIf(i == elements.end(),
                      false,
                      500,
                      "Element '%.*s' does not exist.",
                      int(path.size()),
                      path.data());

        metadata = &i->second;
        return true;
    }

    /**
     * Return the element ID for an existing element.
     *
     * @param path The Slate element path.
     * @param type_id Expected type of the element.
     * @param[out] element_id Returns the element ID.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element_id(const std::string_view path,
                                     const slate_type_t type_id,
                                     slate_element_t &element_id) const
    {
        SacAbortIfNot(get_element_id(path, type_id, slate_private, element_id),
                      false);
        return true;
    }

    /**
     * Return the element ID for an existing element.
     *
     * @param path The Slate element path.
     * @param type_id Expected type of the element.
     * @param access_elevation Elevate all elements to this level of
     *                         access (use slate_primitive to keep
     *                         original access level).
     * @param[out] element_id Returns the element ID.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element_id(const std::string_view path,
                                     const slate_type_t type_id,
                                     const slate_elem_access_t access_elevation,
                                     slate_element_t &element_id) const
    {
        element_id = slate_element_default;

        const SlateElementMetadata *metadata = nullptr;
        SacAbortIfNot(get_element(path, metadata), false);

        if (metadata->type_id != type_id)
        {
            const slate_type_info_t *actual_type_info = nullptr;
            SacAbortIfNot(slate_type_info_utils::get_type_info(
                              metadata->type_id, actual_type_info),
                          false);

            const slate_type_info_t *expected_type_info = nullptr;
            SacAbortIfNot(slate_type_info_utils::get_type_info(
                              type_id, expected_type_info),
                          false);

            SacPrefix();
            dbnprintf(500,
                      ": Element '%s' is not the requested type. Expected "
                      "%hu (%s), got %hu (%s)\n",
                      std::string(path).c_str(),
                      type_id,
                      expected_type_info->name.c_str(),
                      metadata->type_id,
                      actual_type_info->name.c_str());
            return false;
        }

        SacAbortIfNot(
            build_element_id(path, *metadata, access_elevation, element_id),
            false);
        return true;
    }

    /**
     * Construct the element ID for an existing element, elevating access
     * permissions if needed.
     *
     * @param path The Slate element path.
     * @param metadata The Slate element metadata.
     * @param access_elevation Elevate all elements to this level of
     *                         access (use slate_primitive to keep
     *                         original access level).
     * @param[out] element_id Returns the element ID.
     *
     * @return True on success.
     */
    bool
    SlateLayout::build_element_id(const std::string_view path,
                                  const SlateElementMetadata &metadata,
                                  const slate_elem_access_t access_elevation,
                                  slate_element_t &element_id) const
    {
        element_id = slate_element_default;
        SlatePathMap::const_iterator it = elements.find(path);
        SacAbortIf(it == elements.end(), false);
        const slate_index_t idx = elements.iterator_to_id(it);

        const slate_elem_access_t access =
            std::max(metadata.access_policy, access_elevation);

        slate_element_t id;
        SacAbortIfNot(slate_id_buildup(id,
                                       false /* may_write */,
                                       false /* has_vaidator */,
                                       metadata.shard,
                                       metadata.value_offset,
                                       idx),
                      false);
        const bool has_validator = idx_to_validator.count(idx) > 0;

        switch (access)
        {
        case slate_private:
            SacPrefix();
            dbnprintf(500,
                      ": Element '%.*s' is private.\n",
                      int(path.size()),
                      path.data());
            return false;

        case slate_read_only:
            SacAbortIfNot(slate_id_buildup(element_id,
                                           false,
                                           has_validator,
                                           metadata.shard,
                                           metadata.value_offset,
                                           idx),
                          false);
            return true;

        case slate_read_write:
            SacAbortIfNot(slate_id_buildup(element_id,
                                           true,
                                           has_validator,
                                           metadata.shard,
                                           metadata.value_offset,
                                           idx),
                          false);
            return true;

        default:
            SacPrefix();
            dbnprintf(500,
                      ": Element '%.*s' invalid access policy.\n",
                      int(path.size()),
                      path.data());
            return false;
        }
    }

    /**
     * Get the access policy of an element, given its ID.
     *
     * @param element_id ID of the element.
     * @param type_id Type ID of the element.
     * @param[out] access_policy Returns the element's access policy.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element_access_policy(
        const slate_element_t element_id,
        const slate_type_t type_id,
        slate_elem_access_t &access_policy) const
    {
        access_policy = slate_private;

        std::string_view path;
        const SlateElementMetadata *metadata;
        SacAbortIfNot(get_element(element_id, type_id, path, metadata), false);

        access_policy = metadata->access_policy;

        return true;
    }

    /**
     * Get the type of an element, given its path.
     *
     * @param path The Slate element path.
     * @param[out] type_id Returns the element's type ID.
     *
     * @return True on success.
     */
    bool SlateLayout::get_element_type(const std::string_view path,
                                       slate_type_t &type_id) const
    {
        type_id = slate_type_invalid;

        const SlateElementMetadata *metadata = nullptr;
        SacAbortIfNot(get_element(path, metadata), false);

        type_id = metadata->type_id;

        return true;
    }

    /**
     * Get the full path of an element, given its ID and type.
     *
     * @param element_id ID of the element.
     * @param[out] path Returns the path of the element with the
     *             given ID
     *
     * @return True on success.
     */
    bool SlateLayout::get_element_path(const slate_element_t element_id,
                                       std::string_view &path) const
    {
        SacAbortIfNot(slate_id_is_valid(element_id), false);

        const slate_index_t idx = slate_id_index(element_id);
        const SlatePathMap::const_iterator it = elements.id_to_iterator(idx);
        SacAbortIf(it == elements.end(), false);
        path = it->first;

        return true;
    }

    /**
     * Get the full path of an element, given its ID and type. If multiple Slate
     * element paths match the given ID, returns the path of the element that
     * was created first.
     *
     * @param element_id ID of the element.
     * @param type_id Type ID of the element.
     * @param[out] path Returns the path of the first element created with the
     *             given ID and type.
     *
     * @return True on success.
     */
    bool SlateLayout::get_first_element_path(const slate_element_t element_id,
                                             const slate_type_t type_id,
                                             std::string_view &path) const
    {
        const SlateElementMetadata *metadata;
        SacAbortIfNot(get_element(element_id, type_id, path, metadata), false);

        return true;
    }

    /**
     * Get the memory buffer of the initial value of an element.
     *
     * @param element_id ID of the element whose initial value to query.
     * @param type_id Type ID of the element.
     * @param[out] data The memory buffer of the element value.
     *
     * @return True on success.
     */
    bool
    SlateLayout::get_element_initial_memory(const slate_element_t element_id,
                                            const slate_type_t type_id,
                                            B2c &data) const
    {
        data = B2c();

        std::string_view path;
        const SlateElementMetadata *metadata;
        SacAbortIfNot(get_element(element_id, type_id, path, metadata), false);

        data =
            B2c(initial_values[metadata->shard].data() + metadata->value_offset,
                metadata->value_size);

        return true;
    }

    /**
     * Query the slate validator associated with the element ID. If no validator
     * is associated with the element ID, the function succeeds and a default
     * slate_validator_t is returned.
     *
     * @param element_id ID of the element to get the validator for.
     * @param type_id Type ID of the element.
     *
     * @return The validator for the element or a default slate_validator_t if
     *         none is available.
     */
    slate_validator_t
    SlateLayout::get_element_validator(const slate_element_t element_id,
                                       const slate_type_t type_id) const
    {
        const auto found = idx_to_validator.find(slate_id_index(element_id));
        if (found != idx_to_validator.end())
        {
            return found->second;
        }
        else
        {
            return slate_validator_t {};
        }
    }

    /**
     * Returns true if there is an element at this path.
     *
     * @param path Check for an element at this path.
     *
     * @return True if there is an element at this path.
     */
    bool SlateLayout::path_exists(const std::string_view path) const
    {
        return elements.find(path) != elements.end();
    }

    /**
     * Returns true if an element with the given path and type ID exists.
     *
     * @param path Check for an element at this path.
     * @param type_id Check against this type.
     *
     * @return True if path exists and type matches.
     */
    bool SlateLayout::element_exists(const std::string_view path,
                                     const slate_type_t type_id) const
    {
        const auto i = elements.find(path);
        if (i == elements.end())
        {
            return false;
        }

        return i->second.type_id == type_id;
    }

    /**
     * Check for write privileges.
     *
     * @param element_id Check this Slate ID.
     *
     * @return True if \a element_id has write privileges.
     */
    bool SlateLayout::can_write(const slate_element_t element_id) const
    {
        return slate_id_can_write(element_id);
    }

    /**
     * Check if an element has a validator.
     *
     * @param element_id Check this Slate ID.
     *
     * @return True if \a element_id has a validator.
     */
    bool SlateLayout::must_validate(const slate_element_t element_id) const
    {
        return slate_id_has_validator(element_id);
    }

    /**
     * Get the size of the given shard.
     *
     * @param shard Shard to get size of.
     * @param[out] size Size of the shard.
     *
     * @return True on success.
     */
    bool
    SlateLayout::get_shard_size(const slate_shard_t shard, size_t &size) const
    {
        SacAbortIfNot(shard < num_slate_shard_t, false);

        size = shard_size[shard];

        return true;
    }

    /**
     * Finalize layout. This will delete build time metadata.
     *
     * @return True on success.
     */
    bool SlateLayout::finalize()
    {
        for (size_t shard_idx = 0; shard_idx < num_slate_shard_t; ++shard_idx)
        {
            shard_free_list[shard_idx].clear();
            initial_values[shard_idx] = AlignedBuffer();
        }

        elements.clear();
        string_pool.release();

        return true;
    }

    /**
     * Allocate a copy of \ref _path.
     *
     * @param _path The element path.
     *
     * @return A copy of \ref _path.
     */
    std::string_view SlateLayout::allocate_path(const std::string_view _path)
    {
        std::string_view result;

        SacMsgAbortIf(_path.empty(),
                      result,
                      500,
                      "Cannot create element '%.*s', it is reserved as the "
                      "Slate root namespace.\n",
                      int(_path.size()),
                      _path.data());

        SacAbortIfNot(validate_slate_path(_path), result);

        /*
         * Allocate memory for the Slate path and initialize std::string_view.
         * The pointed memory will be cleaned up in the destructor.
         */
        char *ptr = (char *)string_pool.allocate(_path.size(), 1);
        SacAbortIfNot(ptr, result);
        memcpy(ptr, _path.data(), _path.size());

        return std::string_view(ptr, _path.size());
    }

    /**
     * Register the given element and create the element id.
     *
     * @param path The path of the new element. May not be a child or the
     *        parent of another element.
     * @param metadata The new element metadata.
     * @param validator The on-write validator to use.
     * @param[out] element_id Receives the element ID.
     *
     * @return True on success.
     */
    bool
    SlateLayout::add_element_and_build_id(const std::string_view path,
                                          SlateElementMetadata metadata,
                                          const slate_validator_t &validator,
                                          slate_element_t &element_id)
    {
        const auto copied_path = allocate_path(path);
        SacAbortIf(copied_path.empty(), false);

        dbvnprintf(4,
                   250,
                   "SlateLayout allocated path `%.*s`\n",
                   static_cast<int>(copied_path.size()),
                   copied_path.data());

        /*
         * Register the element.
         */
        const auto &[it, inserted] =
            elements.insert(std::make_pair(copied_path, std::move(metadata)));
        SacMsgAbortIfNot(inserted,
                         false,
                         500,
                         "Element '%.*s' already exists.\n",
                         int(path.size()),
                         path.data());
        const slate_index_t idx = elements.iterator_to_id(it);

        /*
         * Return the read/write element ID, unless this element is in
         * the static shard.
         */
        slate_element_t id;
        SacAbortIfNot(slate_id_buildup(id,
                                       metadata.shard != shard_static,
                                       !validator.is_noop(),
                                       metadata.shard,
                                       metadata.value_offset,
                                       idx),
                      false);
        SacAbortIfNot(slate_id_is_valid(id), false);

        /*
         * Register the validator.
         */
        if (!validator.is_noop())
        {
            SacAbortIfNot(idx_to_validator.emplace(idx, validator).second,
                          false);
        }

        element_id = id;
        return true;
    }

} /* end namespace Drone */