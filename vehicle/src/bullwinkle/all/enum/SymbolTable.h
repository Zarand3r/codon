/**
 * @author Richard Bao
 *
 * SymbolTable — a bijective name<->value reflection table for enum symbols.
 *
 * This is build-phase / diagnostic reflection metadata (mapping an enum's
 * integer values to human-readable names for logging, config-file parsing, and
 * cross-string consistency checks). It is NOT a runtime hot-path structure: it
 * uses node-based std containers deliberately, and is only touched during Slate
 * construction, command/state-machine setup, and combiner validation.
 *
 * Callers: `slate_shard_t_sym`, `slate_elem_access_t_sym` (Slate reflection),
 * `StateMachine::state_sym`/`ctask_sym`, and `SlateCombiner` (compares two
 * peers' tables for agreement, then dump()s both on mismatch).
 */

#ifndef SLATE_ENUM_SYMBOL_TABLE_H
#define SLATE_ENUM_SYMBOL_TABLE_H

#include "src/bullwinkle/all/core/drone_types.h"

#include <map>
#include <string>

namespace Drone
{
    /**
     * A bijective mapping between symbol names and unsigned integer values.
     *
     * Every name maps to exactly one value and vice versa; add() rejects a
     * collision on either side rather than overwriting, so a table is always a
     * one-to-one relation. Two tables compare equal iff they hold the same set
     * of name<->value pairs (insertion order is irrelevant).
     */
    class SymbolTable
    {
    public:
        /**
         * Register a name<->value pair.
         *
         * @param name  The symbol name.
         * @param value The integer value.
         *
         * @return True if inserted. False (and no change) if either the name or
         *         the value is already present.
         */
        bool add(const std::string &name, const uint value);

        /**
         * Look up the value bound to a name.
         *
         * @param name      The symbol name.
         * @param[out] value Set to the bound value on success; untouched on miss.
         *
         * @return True if the name is present.
         */
        bool raw_get(const std::string &name, uint &value) const;

        /**
         * Look up the name bound to a value.
         *
         * @param value     The integer value.
         * @param[out] name  Set to the bound name on success; untouched on miss.
         *
         * @return True if the value is present.
         */
        bool raw_get(const uint value, std::string &name) const;

        /**
         * Convenience name lookup for messages.
         *
         * @param value The integer value.
         *
         * @return The bound name, or an empty string if the value is absent.
         *         Never asserts — safe to call directly inside a log format.
         */
        std::string get(const uint value) const;

        /**
         * @return The number of registered pairs.
         */
        size_t size() const { return name_to_value.size(); }

        /**
         * Visit every (value, name) pair in ascending value order.
         */
        template <typename Fn>
        void visit(Fn &&fn) const
        {
            for (const auto &pair : value_to_name)
            {
                fn(pair.first, pair.second);
            }
        }

        /**
         * Print the table's contents for diagnostics.
         */
        void dump() const;

        bool operator==(const SymbolTable &b) const;
        bool operator!=(const SymbolTable &b) const { return !(*this == b); }

    private:
        std::map<std::string, uint> name_to_value{};
        std::map<uint, std::string> value_to_name{};
    };
} /* end namespace Drone */

#endif /* SLATE_ENUM_SYMBOL_TABLE_H */
