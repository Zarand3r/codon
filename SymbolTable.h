Symbol Table

/**
 * @author Chris Sloan
 * @date   07/19/04
 */
#ifndef SYMBOLTABLE_H
#define SYMBOLTABLE_H
#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/core/stl_util.h"
#include "src/bullwinkle/all/enum/auto_enum.h"
#include <map>
#include <stdio.h>
#include <string>
    namespace Drone
{
    /**
     * A map of strings to integers and back based off of the contents
     * of a data file.
     *
     * This class reads its data from files, maps in either direction,
     * and has an optional fallback mechanism.
     *
     * When looking up a value, we first check the maps.  If that
     * fails, we use the fallback mechanism (if enabled), and if that
     * fails, we return a default value.
     *
     * The constructor is passed the default values and is used to
     * enable the fallback mechanism.
     *
     * If the the fallback mechanism is used, strings like "10" or
     * "0xa" would be converted to the integer 10 using
     * Drone::string_to_int().  Similarly, integers are converted to
     * string representations if the fallback is enabled.
     */
    class SymbolTable
    {
    public:
        SymbolTable(bool _use_fallback = false, int _default_int = -1,
                    const std::string _default_string = "<unknown>");
        SymbolTable(const std::string &fname, bool _use_fallback = false,
                    int _default_int = -1,
                    const std::string _default_string = "<unknown>");
        SymbolTable(const string_table_entry *st, int st_size,
                    bool _use_fallback = false, int _default_int = -1,
                    const std::string _default_string = "<unknown>");
        virtual ~SymbolTable();
        bool read(const std::string &fname);
        bool read(FILE *in);
        bool read(const string_table_entry *st, uint st_size);
        bool empty() const;
        bool clear_reads();
        bool copy_reads(const SymbolTable &other);
        bool raw_get(const std::string &s, int &i) const;
        bool raw_get(const std::string &s, int &i, bool _use_fallback) const;
        bool raw_get(const std::string &s, uint &i) const;
        bool raw_get(const std::string &s, uint &i, bool _use_fallback) const;
        bool raw_get(int i, std::string &s) const;
        bool raw_get(int i, std::string &s, bool _use_fallback) const;
        int get(std::string s) const;
        int get(std::string s, int def) const;
        std::string get(int i) const;
        std::string get(int i, const std::string &def) const;
        bool contains(int i) const;
        bool contains(const std::string &s) const;
        const str_int_m &get_s2i_map() const;
        const int_str_m &get_i2s_map() const;
        bool operator==(const SymbolTable &other) const;
        bool operator!=(const SymbolTable &other) const;
        virtual bool add(const std::string &s, int i);
        bool dump(std::string name = "SymbolTable") const;

    protected:
        virtual bool internal_add(const std::string &s, int i);
        str_int_m to_int_table;
        int_str_m to_string_table;

    private:
        const bool use_fallback;
        const int default_int;
        const std::string default_string;
    };
} /* end namespace Drone */
#endif /* SYMBOLTABLE_H */