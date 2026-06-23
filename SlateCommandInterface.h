/**
 * @author Josh Sulkin
 * @date 02/07/2017
 */
#ifndef SLATE_COMMAND_INTERFACE_H
#define SLATE_COMMAND_INTERFACE_H
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/external_command_util.h"
#include "src/bullwinkle/all/runtime.h"
#include "src/bullwinkle/all/static_vector.h"
#include <string>
#include <unordered_map>
namespace Drone
{
    /**
     * A runtime interface to Slate that allows elements to be set by name or
     * hash. This is useful for commanding.
     *
     * Currently only integer and floating point elements are supported.
     */
    class SlateCommandInterface
    {
    public:
        /**
         * A union type to store the value of a single device in a multi
         * command. It can be either an INT64 or a double. A given slate
         * element must always be the same type, so we never need to support
         * both values simultaneously.
         */
        union multi_command_value_t
        {
            INT64 as_int;
            double as_dbl;
            multi_command_value_t(INT64 value) : as_int(value) {}
            multi_command_value_t(double value) : as_dbl(value) {}
        };
        /**
         * Enum to indicate if given command contains an INT64 or double type.
         */
        enum multi_command_type_t
        {
            Int,
            Dbl,
        };
        /**
         * Stores commands for use in multi command. Multi commands lookup
         * devices via their hash and can be used to set INT64 or double types.
         */
        struct multi_command_t
        {
            multi_command_t(const multi_command_type_t _type,
                            const external_command_name_hash_t &_hash,
                            multi_command_value_t _value)
                : type(_type), hash(_hash), value(_value)
            {}
            multi_command_t()
                : type(multi_command_type_t::Int), hash(), value(0LL)
            {}
            /**
             * The type of the command - either an int64 or a double.
             */
            multi_command_type_t type;
            /**
             * External hash of the device.
             */
            external_command_name_hash_t hash;
            /**
             * Value to set / storage for the previous value of the device. We
             * use this memory as a cache in case we need to roll back the
             * entire batch of commands.
             */
            multi_command_value_t value;
        };
        /**
         * A vector of multi commands. We should never accept more than 50
         * commands.
         */
        typedef static_vector<multi_command_t, 50> multi_command_v;
        SlateCommandInterface();
        bool init(SlateBuilder builder);
        bool init(SlateBuilder builder, const std::string &prefix);
        bool set_int_by_name(const std::string &name, const INT64 value,
                             bool &success) RUNTIME;
        bool set_int_by_hash(const external_command_name_hash_t &hash,
                             const INT64 value, bool &success) RUNTIME;
        bool set_fp_by_name(const std::string &name, const double value,
                            bool &success) RUNTIME;
        bool set_fp_by_hash(const external_command_name_hash_t &hash,
                            const double value, bool &success) RUNTIME;
        bool get_int_by_hash(const external_command_name_hash_t &hash,
                             bool &success, INT64 &value) const;
        bool get_fp_by_hash(const external_command_name_hash_t &hash,
                            bool &success, double &value) const;
        bool get_type_by_hash(const external_command_name_hash_t &hash,
                              bool &success, multi_command_type_t &type) const;
        bool set_multi_by_hash(multi_command_v &cmds, bool &success) RUNTIME;

    private:
        /**
         * Stores the ID and type of an element without requiring a template.
         */
        struct element_t
        {
            /**
             * Slate ID of the given element.
             */
            slate_element_t id = slate_element_default;
            /**
             * Slate type of the given element.
             */
            slate_type_t type = slate_type_invalid;
        };
        /**
         * Set a Slate element.
         *
         * @note This method will return true even if the requested element was
         *       not successfully set. You must check the "success" flag to
         *       determine whether the set was completed.
         *
         * @tparam K Type of the key for looking up the Slate element.
         * @tparam T Type of the Slate element value.
         *
         * @param key The lookup key.
         * @param value Value to set.
         * @param[out] success True if the element was successfully set.
         *
         * @return True, unless there was a programming error.
         */
        template <typename K, typename T>
        bool set(const K &key, const T &value, bool &success) RUNTIME
        {
            success = false;
            FswAbortIfNot(is_init, false);
            element_t elem;
            external_command_name_hash_t hash(key);
            if (!lookup(key, elem))
            {
                return true;
            }
            if (!validate_element(elem, hash))
            {
                return true;
            }
            success = set(elem, hash, value);
            return true;
        }
        /**
         * Get the current value of a Slate element.
         *
         * @note This method will return true even if the requested element does
         *       not exist, or cannot be converted to the requested type. You
         *       must check the "success" flag to determine whether the get was
         *       actually successful.
         *
         * @tparam K Type of the key for looking up the Slate element. Only
         *           lookups using external_command_name_hash_t are supported.
         * @tparam T Type of the Slate element value.
         *
         * @param key The lookup key.
         * @param[out] success True if the element was found and is convertible
         *                     to type T.
         * @param[out] value If success is true, returns the value of the
         *                   requested element.
         *
         * @return True, unless there was a programming error.
         */
        template <typename K, typename T>
        bool get(const K &key, bool &success, T &value) const
        {
            success = false;
            value = T();
            FswAbortIfNot(is_init, false);
            element_t elem;
            const external_command_name_hash_t hash(key);
            if (!lookup(hash, elem))
            {
                return true;
            }
            success = get(elem, hash, value);
            return true;
        }
        bool set(const element_t &elem, external_command_name_hash_t hash,
                 INT64 value) RUNTIME;
        bool set(const element_t &elem, external_command_name_hash_t hash,
                 const double value) RUNTIME;
        bool get(const element_t &elem, external_command_name_hash_t hash,
                 INT64 &value) const;
        bool get(const element_t &elem, external_command_name_hash_t hash,
                 double &value) const;
        bool lookup(const std::string &name, element_t &elem) const;
        bool lookup(const external_command_name_hash_t &hash,
                    element_t &elem) const;
        static bool validate_element(const element_t &elem,
                                     external_command_name_hash_t hash);
        bool insert_element(const slate_element_t id, const slate_type_t type,
                            std::string_view name);
        /**
         * A convenience type for a map that allows for looking up elements by
         * hash.
         */
        using element_m = std::unordered_map<UINT64, element_t>;
        /**
         * Map of element name hashes to element info.
         */
        element_m elements;
        /**
         * Runtime Slate.
         */
        INFRASTRUCTURE(Slate) slate;
        /**
         * True if init() has been successfully called.
         */
        bool is_init;
    };
} /* end namespace Drone */
#endif /* SLATE_COMMAND_INTERFACE_H */
