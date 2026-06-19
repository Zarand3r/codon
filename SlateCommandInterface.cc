/**
 * @author Josh Sulkin
 * @date 02/07/2017
 */
#include "src/bullwinkle/all/SlateCommandInterface.h"
#include "src/bullwinkle/all/core/math/sxmath.h"
#include "src/bullwinkle/all/core/stl_util.h"
#include <limits>
namespace Drone
{
    namespace
    {
        /*
         * Type IDs.
         *
         * TODO: #12544: Remove ulong_type once UINT64 is an unsigned long
         * on 64-bit targets.
         */
        constexpr slate_type_t bool_type = slate_type_fixed_id_bool;
        constexpr slate_type_t int8_type = slate_type_fixed_id_int8;
        constexpr slate_type_t int16_type = slate_type_fixed_id_int16;
        constexpr slate_type_t int32_type = slate_type_fixed_id_int32;
        constexpr slate_type_t int64_type = slate_type_fixed_id_int64;
        constexpr slate_type_t uint8_type = slate_type_fixed_id_uint8;
        constexpr slate_type_t uint16_type = slate_type_fixed_id_uint16;
        constexpr slate_type_t uint32_type = slate_type_fixed_id_uint32;
        constexpr slate_type_t ulong_type = slate_type_fixed_id_ulong;
        constexpr slate_type_t uint64_type = slate_type_fixed_id_uint64;
        constexpr slate_type_t float_type = slate_type_fixed_id_float;
        constexpr slate_type_t double_type = slate_type_fixed_id_double;
        /**
         * Validates that an element's value represented as an INT64 is within
         * the bounds of the type T. If it is not, a warning will be printed and
         * false will be returned.
         *
         * @tparam T The underlying integer or bool type of the element.
         *
         * @param value The value of the element represented as an INT64.
         * @param hash The hash of the element.
         *
         * @return True on success.
         */
        template <typename T>
        bool validate_int_element_bounds(INT64 value,
                                         external_command_name_hash_t hash)
        {
            /*
             * The minimum value for a UINT32 is specified to be the same as the
             * minimum value for an INT32. This is a special case that ensures
             * that if a 32-bit int was upcast to a 64-bit int, it can still be
             * used to set all the bits of a 32-bit Slate element (i.e. by using
             * negative values).
             */
            using min_type_t =
                std::conditional_t<std::is_same<T, UINT32>::value, INT32, T>;
            constexpr INT64 min_value =
                static_cast<INT64>(std::numeric_limits<min_type_t>::min());
            constexpr INT64 max_value =
                static_cast<INT64>(std::numeric_limits<T>::max());
            if (value < min_value || value > max_value)
            {
                dbnprintf(
                    200,
                    "Slate Command Error: '%08llx' - Value %lld outside bounds "
                    "[%lld,%lld].\n",
                    hash.value, value, min_value, max_value);
                return false;
            }
            return true;
        }
    } // namespace
    /**
     * Constructor.
     */
    SlateCommandInterface::SlateCommandInterface()
        : elements(), slate(), is_init(false)
    {}
    /**
     * Initialize.
     *
     * @param builder A built super Slate.
     *
     * @return True on success.
     */
    bool SlateCommandInterface::init(SlateBuilder builder)
    {
        SacAbortIfNot(init(builder, ""), false);
        return true;
    }
    /**
     * Initialize.
     *
     * @param builder A built super Slate.
     * @param prefix Store all element names with and without this prefix. This
     *        allows commanding them with or without providing the prefix.
     *
     * @note The prefix needs to match the end of the subtree path of the
     *       provided SlateBuilder. For example, if the subtree path is "a.b.c",
     *       these are valid prefixes: "a.b.c", "b.c", "c", "".
     *
     * @return True on success.
     */
    bool SlateCommandInterface::init(SlateBuilder builder,
                                     const std::string &prefix)
    {
        SacAbortIf(is_init, false);
        /*
         * The prefix should match the end of the builder path.
         */
        const std::string &subtree = builder.get_subtree_path();
        SacAbortIfNot(strsuffix(subtree, prefix), false);
        /*
         * Make sure this is a built super SlateBuilder. This is to enforce that
         * all elements are created and we can write to all of them.
         */
        SacAbortIfNot(builder.is_built(), false);
        SacAbortIfNot(builder.is_super_slate(), false);
        /*
         * Get the runtime Slate.
         */
        slate = builder.slate(slate_no_validation);
        /*
         * Lookup all the elements in this builder.
         */
        const SlateLayout &layout = slate.get_layout();
        const SlatePathMap &path_map = layout.get_elements();
        /*
         * Set the load factor for the hash table to something that will reduce
         * the number of collisions while not wasting too much memory. Also,
         * reserve the size to make inserting elements faster (i.e. by avoiding
         * re-hashes).
         */
        elements.max_load_factor(0.7f);
        /*
         * Iterate over every element.
         */
        for (const auto &[path, metadata] : path_map)
        {
            /*
             * Store every integer and floating point element.
             */
            if (metadata.type_id == bool_type ||
                metadata.type_id == int8_type ||
                metadata.type_id == int16_type ||
                metadata.type_id == int32_type ||
                metadata.type_id == int64_type ||
                metadata.type_id == uint8_type ||
                metadata.type_id == uint16_type ||
                metadata.type_id == uint32_type ||
                metadata.type_id == ulong_type ||
                metadata.type_id == uint64_type ||
                metadata.type_id == float_type ||
                metadata.type_id == double_type)
            {
                std::string_view rel_path;
                if (!slate_rel_path(subtree, path, rel_path))
                {
                    continue;
                }
                slate_element_t element_id;
                SacAbortIfNot(layout.build_element_id(
                                  path, metadata, slate_read_write, element_id),
                              false);
                SacAbortIfNot(
                    insert_element(element_id, metadata.type_id, rel_path),
                    false);
                if (!prefix.empty())
                {
                    SacAbortIfNot(
                        insert_element(element_id, metadata.type_id, path),
                        false);
                }
            }
        }
        /*
         * Make sure that none of the buckets in the hash table contain too
         * many elements. This bounds the worst case lookup. The bound is set by
         * the expected worst case number of hops to search in a balanced binary
         * tree. Note that the bound can be no less than that which would be
         * calculated from a Slate with 100 elements, to prevent a very small
         * Slate from failing the check.
         */
        size_t max_bucket_size = 0;
        for (size_t i = 0; i < elements.bucket_count(); ++i)
        {
            const size_t bucket_size = elements.bucket_size(i);
            max_bucket_size = std::max(max_bucket_size, bucket_size);
        }
        SacAbortIf(elements.size() == 0, false);
        const size_t max_allowed_bucket_size =
            std::max(static_cast<size_t>(sx_floor(sx_log2(100.0))),
                     static_cast<size_t>(sx_floor(
                         sx_log2(static_cast<double>(elements.size())))));
        SacAbortIfNotOpUint(max_bucket_size, <=, max_allowed_bucket_size,
                            false);
        is_init = true;
        return true;
    }
    /**
     * Set an integer element. Lookup by name and set from an INT64.
     *
     * @note This method will return true even if the requested element was not
     *       successfully set. You must check the "success" flag to determine
     *       whether the set was completed.
     *
     * @param name Name of the element.
     * @param value Value to set.
     * @param[out] success True if the element was successfully set.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::set_int_by_name(const std::string &name,
                                                const INT64 value,
                                                bool &success) RUNTIME
    {
        SacAbortIfNot(set(name, value, success), false);
        return true;
    }
    /**
     * Set an integer element. Lookup by hash and set from an INT64.
     *
     * @note This method will return true even if the requested element was not
     *       successfully set. You must check the "success" flag to determine
     *       whether the set was completed.
     *
     * @param hash Hash of the element name.
     * @param value Value to set.
     * @param[out] success True if the element was successfully set.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::set_int_by_hash(
        const external_command_name_hash_t &hash, const INT64 value,
        bool &success) RUNTIME
    {
        SacAbortIfNot(set(hash, value, success), false);
        return true;
    }
    /**
     * Set a floating point element. Lookup by name.
     *
     * @note This method will return true even if the requested element was not
     *       successfully set. You must check the "success" flag to determine
     *       whether the set was completed.
     *
     * @param name Name of the element.
     * @param value Value to set.
     * @param[out] success True if the element was successfully set.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::set_fp_by_name(const std::string &name,
                                               const double value,
                                               bool &success) RUNTIME
    {
        SacAbortIfNot(set(name, value, success), false);
        return true;
    }
    /**
     * Set a floating point element. Lookup by hash.
     *
     * @note This method will return true even if the requested element was not
     *       successfully set. You must check the "success" flag to determine
     *       whether the set was completed.
     *
     * @param hash Hash of the element name.
     * @param value Value to set.
     * @param[out] success True if the element was successfully set.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::set_fp_by_hash(
        const external_command_name_hash_t &hash, const double value,
        bool &success) RUNTIME
    {
        SacAbortIfNot(set(hash, value, success), false);
        return true;
    }
    /**
     * Get the current value of an integer element. Lookup by hash.
     *
     * @note This method will return true even if the requested element does
     *       not exist, or cannot be converted to the requested type. You
     *       must check the "success" flag to determine whether the get was
     *       actually successful.
     *
     * @param hash Hash of the element name.
     * @param[out] success True if the element was found and is convertible to
     *                     an INT64.
     * @param[out] value Value of the element, as an INT64.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::get_int_by_hash(
        const external_command_name_hash_t &hash, bool &success,
        INT64 &value) const
    {
        SacAbortIfNot(get(hash, success, value), false);
        return true;
    }
    /**
     * Get the current value of a floating point element. Lookup by hash.
     *
     * @note This method will return true even if the requested element does
     *       not exist, or cannot be converted to the requested type. You
     *       must check the "success" flag to determine whether the get was
     *       actually successful.
     *
     * @param hash Hash of the element name.
     * @param[out] success True if the element was found and is convertible to a
     *                     double.
     * @param[out] value Value of the element, as a double.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::get_fp_by_hash(
        const external_command_name_hash_t &hash, bool &success,
        double &value) const
    {
        SacAbortIfNot(get(hash, success, value), false);
        return true;
    }
    /**
     * Determine if a slate element is a double or float. The lookup is done by
     * hash.
     *
     * @note This method will return true even if the requested element does
     *       not exist. You must check the "success" flag to determine whether
     *       the get was actually successful.
     *
     * @param hash Hash of the element name in Slate.
     * @param[out] success True if the element was found.
     * @param[out] type Type of the element.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::get_type_by_hash(
        const external_command_name_hash_t &hash, bool &success,
        multi_command_type_t &type) const
    {
        element_t elem;
        const bool element_exists = lookup(hash, elem);
        success = false;
        /*
         * If lookup fails, return with success set to false.
         */
        if (!element_exists)
        {
            return true;
        }
        /*
         * Set the output type to integer if the slate element can be
         * represented as an INT64.
         */
        else if (elem.type == bool_type || elem.type == int8_type ||
                 elem.type == int16_type || elem.type == int32_type ||
                 elem.type == int64_type || elem.type == uint8_type ||
                 elem.type == uint16_type || elem.type == uint32_type ||
                 elem.type == ulong_type || elem.type == uint64_type)
        {
            type = Int;
            success = true;
        }
        /*
         * Set the output type to double if the slate element can be represented
         * as a double.
         */
        else if (elem.type == double_type || elem.type == float_type)
        {
            type = Dbl;
            success = true;
        }
        return true;
    }
    /**
     * Set multiple elements by hash. This command is atomic: if any individual
     * set is rejected according to the rules of a regular single-element
     * command, all sets will be rolled back.
     *
     * @note This method will return true even if the requested elements were
     *       not successfully set. You must check the "success" flag to
     *       determine whether the set was completed.
     *
     * @param cmds A list of the commands to apply.
     * @param[out] success True if the elements were successfully set.
     *
     * @return True, unless there was a programming error.
     */
    bool SlateCommandInterface::set_multi_by_hash(multi_command_v &cmds,
                                                  bool &success) RUNTIME
    {
        size_t cmd_index = 0;
        /*
         * Iterate through all commands and try to set their values. If any
         * individual set fails, we undo all previous successful sets (not
         * including the one that failed).
         *
         * We achieve this in a memory-efficient way by storing the current
         * slate value for each device in its corresponding multi_command_t
         * before each set attempt - if any attempt fails, we simply replace
         * all the previous values.
         */
        for (; cmd_index < cmds.size(); cmd_index++)
        {
            element_t elem;
            auto &command = cmds[cmd_index];
            const bool element_exists = lookup(command.hash, elem);
            /*
             * We can't find this element, so break and roll back from here.
             */
            if (!element_exists)
            {
                break;
            }
            /*
             * We looked up the element, so now record its value and attempt
             * to set the new one.
             */
            if (command.type == multi_command_type_t::Int)
            {
                const INT64 to_set = command.value.as_int;
                const bool get_completed =
                    get(elem, command.hash, command.value.as_int);
                if (!get_completed || !set(elem, command.hash, to_set))
                {
                    break;
                }
            }
            else if (command.type == multi_command_type_t::Dbl)
            {
                const double to_set = command.value.as_dbl;
                const bool get_completed =
                    get(elem, command.hash, command.value.as_dbl);
                if (!get_completed || !set(elem, command.hash, to_set))
                {
                    break;
                }
            }
            else
            {
                /*
                 * We should never get here, but if we do, break and roll back.
                 */
                SacPrefix();
                dbnprintf(
                    200,
                    ": Received an invalid command type. Must be Int (%d) "
                    "or Double (%d) but got %d.\n",
                    multi_command_type_t::Int, multi_command_type_t::Dbl,
                    command.type);
                break;
            }
        }
        /*
         * If we made it through all commands successfully, we're done.
         */
        if (cmd_index == cmds.size())
        {
            success = true;
            return true;
        }
        /*
         * If we didn't make it through every command, we need to roll back.
         * This is what guarantees the atomicity of the multi command.
         */
        success = false;
        const size_t failed_on = cmd_index;
        {
            const auto &command = cmds[cmd_index];
            dbnprintf(500,
                      "Failed to set an element of the multi command, which "
                      "triggered a rollback.\n"
                      "The element was at index: %zu and has hash 0x%llx.\n",
                      failed_on, command.hash.value);
        }
        /*
         * We roll back all commands up but not including to the one that
         * failed by resetting the slate value we recorded in the corresponding
         * multi_command_t.
         *
         * We only undo sets which have already been successful, which means:
         *
         *  1. The slate element has been looked up successfully.
         *  2. The data type is correct.
         *  3. We were able to set a value.
         *
         * Therefore, there's no expectation that doing those things _again_
         * can fail.
         *
         * However, in case of a pathological validator or programming error,
         * we return false.
         */
        bool rollback_successful = true;
        for (cmd_index = 0; cmd_index < failed_on; cmd_index++)
        {
            const auto &command = cmds[cmd_index];
            bool set_successful = false;
            bool set_completed = false;
            if (command.type == multi_command_type_t::Int)
            {
                set_completed =
                    set(command.hash, command.value.as_int, set_successful);
            }
            else if (command.type == multi_command_type_t::Dbl)
            {
                set_completed =
                    set(command.hash, command.value.as_dbl, set_successful);
            }
            else
            {
                rollback_successful = false;
                SacPrefix();
                dbnprintf(
                    500,
                    ": Failed to reset an element of the multi command, while "
                    "rolling back.\n"
                    "The element was at index: %zu and has hash 0x%llx.\n",
                    cmd_index, command.hash.value);
            }
            rollback_successful &= set_successful && set_completed;
        }
        return rollback_successful;
    }
    /**
     * Set an integer element from an INT64.
     *
     * @param elem The element to set.
     * @param hash The element hash.
     * @param value Value to set.
     *
     * @return True if the value was successfully set.
     */
    bool SlateCommandInterface::set(const element_t &elem,
                                    external_command_name_hash_t hash,
                                    INT64 value) RUNTIME
    {
        SacDebugAssert(is_init);
        if (elem.type == bool_type)
        {
            return validate_int_element_bounds<bool>(value, hash) &&
                   slate.store<bool>(elem.id, value);
        }
        else if (elem.type == int8_type)
        {
            return validate_int_element_bounds<INT8>(value, hash) &&
                   slate.store<INT8>(elem.id, value);
        }
        else if (elem.type == int16_type)
        {
            return validate_int_element_bounds<INT16>(value, hash) &&
                   slate.store<INT16>(elem.id, value);
        }
        else if (elem.type == int32_type)
        {
            return validate_int_element_bounds<INT32>(value, hash) &&
                   slate.store<INT32>(elem.id, value);
        }
        else if (elem.type == int64_type)
        {
            return slate.store<INT64>(elem.id, value);
        }
        else if (elem.type == uint8_type)
        {
            return validate_int_element_bounds<UINT8>(value, hash) &&
                   slate.store<UINT8>(elem.id, value);
        }
        else if (elem.type == uint16_type)
        {
            return validate_int_element_bounds<UINT16>(value, hash) &&
                   slate.store<UINT16>(elem.id, value);
        }
        else if (elem.type == uint32_type)
        {
            return validate_int_element_bounds<UINT32>(value, hash) &&
                   slate.store<UINT32>(elem.id, value);
        }
        else if (elem.type == ulong_type)
        {
            static_assert(sizeof(unsigned long) == 4 ||
                              sizeof(unsigned long) == 8,
                          "invalid size of unsigned long");
            if (sizeof(unsigned long) == 4)
            {
                if (!validate_int_element_bounds<UINT32>(value, hash))
                {
                    return false;
                }
            }
            return slate.store<unsigned long>(elem.id, value);
        }
        else if (elem.type == uint64_type)
        {
            return slate.store<UINT64>(elem.id, value);
        }
        else
        {
            dbnprintf(200,
                      "Slate Command Error: '%08llx' - Not an INT element.\n",
                      hash.value);
        }
        return false;
    }
    /**
     * Set a floating point element.
     *
     * @param elem The element to set.
     * @param hash The element hash.
     * @param value Value to set.
     *
     * @return True if the value was successfully set.
     */
#if defined(__clang__)
    __attribute__((no_sanitize("float-cast-overflow")))
#endif
    bool
    SlateCommandInterface::set(const element_t &elem,
                               external_command_name_hash_t hash,
                               const double value) RUNTIME
    {
        SacDebugAssert(is_init);
        if (elem.type == double_type)
        {
            if (!sx_isfinite(value))
            {
                dbnprintf(200,
                          "Slate Command Error: '%08llx' - Non-finite double "
                          "value.\n",
                          hash.value);
                return false;
            }
            return slate.store<double>(elem.id, value);
        }
        else if (elem.type == float_type)
        {
            const float float_value = static_cast<float>(value);
            if (!sx_isfinite(float_value))
            {
                dbnprintf(200,
                          "Slate Command Error: '%08llx' - Non-finite float "
                          "value.\n",
                          hash.value);
                return false;
            }
            return slate.store<float>(elem.id, float_value);
        }
        else
        {
            dbnprintf(200,
                      "Slate Command Error: '%08llx' - Not a FP element.\n",
                      hash.value);
        }
        return false;
    }
    /**
     * Get the current value of an element and return it as an INT64.
     *
     * @param elem The element to get.
     * @param hash The element hash.
     * @param[out] value Returns the value.
     *
     * @return True if the value was successfully retrieved.
     */
    bool SlateCommandInterface::get(const element_t &elem,
                                    external_command_name_hash_t hash,
                                    INT64 &value) const
    {
        SacDebugAssert(is_init);
        if (elem.type == bool_type)
        {
            value = slate.load<bool>(elem.id);
        }
        else if (elem.type == int8_type)
        {
            value = slate.load<INT8>(elem.id);
        }
        else if (elem.type == int16_type)
        {
            value = slate.load<INT16>(elem.id);
        }
        else if (elem.type == int32_type)
        {
            value = slate.load<INT32>(elem.id);
        }
        else if (elem.type == int64_type)
        {
            value = slate.load<INT64>(elem.id);
        }
        else if (elem.type == uint8_type)
        {
            value = slate.load<UINT8>(elem.id);
        }
        else if (elem.type == uint16_type)
        {
            value = slate.load<UINT16>(elem.id);
        }
        else if (elem.type == uint32_type)
        {
            value = slate.load<UINT32>(elem.id);
        }
        else if (elem.type == ulong_type)
        {
            value = slate.load<unsigned long>(elem.id);
        }
        else if (elem.type == uint64_type)
        {
            value = slate.load<UINT64>(elem.id);
        }
        else
        {
            dbnprintf(200,
                      "Slate Command Error: '%08llx' - Not an INT element.\n",
                      hash.value);
            return false;
        }
        return true;
    }
    /**
     * Get the current value of an element and return it as a floating point
     * (double).
     *
     * @param elem The element to get.
     * @param hash The element hash.
     * @param[out] value Returns the value.
     *
     * @return True if the value was successfully retrieved.
     */
    bool SlateCommandInterface::get(const element_t &elem,
                                    external_command_name_hash_t hash,
                                    double &value) const
    {
        SacDebugAssert(is_init);
        if (elem.type == double_type)
        {
            value = slate.load<double>(elem.id);
        }
        else if (elem.type == float_type)
        {
            value = static_cast<double>(slate.load<float>(elem.id));
        }
        else
        {
            dbnprintf(200,
                      "Slate Command Error: '%08llx' - Not a FP element.\n",
                      hash.value);
            return false;
        }
        return true;
    }
    /**
     * Lookup an element by name.
     *
     * @param name Name of the element.
     * @param[out] elem The element.
     *
     * @return True on success.
     */
    bool SlateCommandInterface::lookup(const std::string &name,
                                       element_t &elem) const
    {
        SacDebugAssert(is_init);
        const external_command_name_hash_t hash(name);
        if (!map_find(elements, hash.value, elem))
        {
            dbnprintf(200, "Slate Command Error: '%s' - Name not found.\n",
                      name.c_str());
            return false;
        }
        return true;
    }
    /**
     * Lookup an element by name hash.
     *
     * @param hash Hash of the element name.
     * @param[out] elem The element.
     *
     * @return True on success.
     */
    bool SlateCommandInterface::lookup(const external_command_name_hash_t &hash,
                                       element_t &elem) const
    {
        SacDebugAssert(is_init);
        if (!map_find(elements, hash.value, elem))
        {
            dbnprintf(200,
                      "Slate Command Error: Name not found "
                      "(hash = '0x%llx').\n",
                      hash.value);
            return false;
        }
        return true;
    }
    /**
     * Validate that an element may be accessed at runtime.
     *
     * @param elem The element.
     * @param hash The element hash.
     *
     * @return True if the element may be accessed at runtime.
     */
    bool
    SlateCommandInterface::validate_element(const element_t &elem,
                                            external_command_name_hash_t hash)
    {
        slate_shard_t shard = shard_static;
        {
            bool may_write = false;
            bool has_validator = false;
            slate_offset_t offset = 0;
            slate_id_breakdown(elem.id, may_write, has_validator, shard,
                               offset);
        }
        if (shard == shard_static)
        {
            dbnprintf(200,
                      "Slate Command Error: '%08llx' - Static shard "
                      "elements may not be commanded.\n",
                      hash.value);
            return false;
        }
        return true;
    }
    /**
     * Insert an element into the map of elements.
     *
     * @param id The element Slate ID.
     * @param type The element Slate type.
     * @param name The element name.
     *
     * @return True on success.
     */
    bool SlateCommandInterface::insert_element(const slate_element_t id,
                                               const slate_type_t type,
                                               std::string_view name)
    {
        SacAbortIf(is_init, false);
        SacAbortIf(name.empty(), false);
        const external_command_name_hash_t hash(name);
        bool inserted =
            elements.insert({hash.value, element_t{.id = id, .type = type}})
                .second;
        if (!inserted)
        {
            SacMsgAbort(false, 200,
                        "'%.*s' has the same hash '0x%llx' as another element.",
                        (int)name.size(), name.data(), hash.value);
        }
        return true;
    }
} /* end namespace Drone */