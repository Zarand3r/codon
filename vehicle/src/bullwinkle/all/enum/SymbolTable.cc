/**
 * @author Richard Bao
 *
 * SymbolTable implementation. See SymbolTable.h for the contract.
 */

#include "src/bullwinkle/all/enum/SymbolTable.h"

#include "src/bullwinkle/all/core/fsw.h"

#include <cstdio>

namespace Drone
{
    bool SymbolTable::add(const std::string &name, const uint value)
    {
        /*
         * Reject a collision on either side so the relation stays one-to-one.
         * A partial insert would corrupt the bijection, so both maps are only
         * touched once both are known to be clear.
         */
        if (name_to_value.count(name) || value_to_name.count(value))
        {
            return false;
        }

        name_to_value.emplace(name, value);
        value_to_name.emplace(value, name);
        return true;
    }

    bool SymbolTable::raw_get(const std::string &name, uint &value) const
    {
        const auto it = name_to_value.find(name);
        if (it == name_to_value.end())
        {
            return false;
        }
        value = it->second;
        return true;
    }

    bool SymbolTable::raw_get(const uint value, std::string &name) const
    {
        const auto it = value_to_name.find(value);
        if (it == value_to_name.end())
        {
            return false;
        }
        name = it->second;
        return true;
    }

    std::string SymbolTable::get(const uint value) const
    {
        const auto it = value_to_name.find(value);
        if (it == value_to_name.end())
        {
            return std::string();
        }
        return it->second;
    }

    void SymbolTable::dump() const
    {
        for (const auto &pair : value_to_name)
        {
            std::printf("  %u -> %s\n", pair.first, pair.second.c_str());
        }
    }

    bool SymbolTable::operator==(const SymbolTable &b) const
    {
        /*
         * The two internal maps are kept mutually consistent by add(), so
         * comparing the name->value direction is sufficient to establish that
         * the full bijection matches.
         */
        return name_to_value == b.name_to_value;
    }
} /* end namespace Drone */
