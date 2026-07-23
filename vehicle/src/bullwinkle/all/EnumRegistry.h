/**
 * @author Richard Bao
 *
 * EnumRegistry — records which Slate elements carry enumerated values and the
 * name<->value SymbolTable describing each, keyed by the element's full path
 * ("channel name"). SlateCombiner uses it to check cross-string enum agreement;
 * telemetry uses it to render names. Build-phase/diagnostic only (cold path).
 *
 * Contract from usage (SlateBuilder.cc:434,505; SlateBuilder.h:728):
 *   register_enum(enum_name, symbol_table, channel_name, strip_prefix) -> bool
 *   get_registered_enum(channel_name, enum_name&, symbol_table&) -> bool (exists)
 *   register_auto_enum<Enum_T>(full_path) -> bool
 *
 * Auto-enums: the original tree generated a SymbolTable per enum. With codegen
 * dropped (plan D3 revised), register_auto_enum<E> resolves E's table through an
 * ADL hook `auto_enum_symbol_table(E)` that each auto-enum's header must provide;
 * instantiating it for an enum without the hook is a compile error at the call
 * site (compiler-enforced, no silent fallback). No present flight code
 * instantiates it yet.
 */

#ifndef ENUM_REGISTRY_H
#define ENUM_REGISTRY_H

#include "src/bullwinkle/all/enum/SymbolTable.h"

#include <map>
#include <string>

namespace Drone
{
    class EnumRegistry
    {
    public:
        /**
         * Record that element `channel_name` carries enum `enum_name` described
         * by `symbol_table`. If `strip_prefix` is non-empty, it is removed from
         * the front of every symbol name (making generated names readable).
         *
         * @return False if the channel already has a registered enum.
         */
        bool register_enum(const std::string &enum_name,
                           const SymbolTable &symbol_table,
                           const std::string &channel_name,
                           const std::string &strip_prefix = std::string())
        {
            if (channels.count(channel_name))
            {
                return false;
            }
            entry_t e;
            e.enum_name = enum_name;
            if (strip_prefix.empty())
            {
                e.table = symbol_table;
            }
            else
            {
                /* Rebuild the table with the prefix stripped from each name
                 * (iterates actual entries — no value range assumed). */
                bool ok = true;
                symbol_table.visit([&](uint v, const std::string &n) {
                    std::string name = n;
                    if (name.compare(0, strip_prefix.size(), strip_prefix) == 0)
                    {
                        name.erase(0, strip_prefix.size());
                    }
                    ok = ok && e.table.add(name, v);
                });
                if (!ok)
                {
                    return false;
                }
            }
            channels.emplace(channel_name, std::move(e));
            return true;
        }

        /**
         * Look up the enum registered for `channel_name`.
         *
         * @return True if one is registered (outputs filled); false otherwise.
         */
        bool get_registered_enum(const std::string &channel_name,
                                 std::string &enum_name,
                                 SymbolTable &symbol_table) const
        {
            const auto it = channels.find(channel_name);
            if (it == channels.end())
            {
                return false;
            }
            enum_name = it->second.enum_name;
            symbol_table = it->second.table;
            return true;
        }

        /**
         * Register an auto-enum element. Resolves Enum_T's reflection through the
         * ADL hook `auto_enum_symbol_table(Enum_T{})`; a missing hook fails to
         * compile at the instantiation site.
         */
        template <typename Enum_T>
        bool register_auto_enum(const std::string &full_path)
        {
            const SymbolTable table = auto_enum_symbol_table(Enum_T{});
            return register_enum(auto_enum_name(Enum_T{}), table, full_path);
        }

    private:
        struct entry_t
        {
            std::string enum_name{};
            SymbolTable table{};
        };

        /* channel (element full path) -> registered enum. Cold-path std::map. */
        std::map<std::string, entry_t> channels{};
    };

} /* end namespace Drone */

#endif /* ENUM_REGISTRY_H */
