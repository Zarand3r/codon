/**
 * @author Chris Sloan
 * @date   07/19/04
 */
#include "src/bullwinkle/all/enum/SymbolTable.h"
#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/core/str_util.h"
#include "src/bullwinkle/all/core/util.h"
#include "src/bullwinkle/all/file/file_parse.h"
namespace Drone
{
    /**
     * Create a new symbol table.
     *
     * @param _use_fallback If true, enable the fallback mechanism.
     *
     * @param _default_int The default integer to return if a string
     * can't be translated.
     *
     * @param _default_string The default string to return if an
     * integer can't be translated.
     */
    SymbolTable::SymbolTable(bool _use_fallback, int _default_int,
                             const std::string _default_string)
        : use_fallback(_use_fallback), default_int(_default_int),
          default_string(_default_string)
    {}
    /**
     * Create a new symbol table from a data file or exit the program
     * if the file can not be read.
     *
     * @param fname The file to read.
     *
     * @param _use_fallback If true, enable the fallback mechanism.
     *
     * @param _default_int The default integer to return if a string
     * can't be translated.
     *
     * @param _default_string The default string to return if an
     * integer can't be translated.
     */
    SymbolTable::SymbolTable(const std::string &fname, bool _use_fallback,
                             int _default_int,
                             const std::string _default_string)
        : use_fallback(_use_fallback), default_int(_default_int),
          default_string(_default_string)
    {
        FswAssert(read(fname));
    }
    SymbolTable::SymbolTable(const string_table_entry *st, int st_size,
                             bool _use_fallback, int _default_int,
                             const std::string _default_string)
        : use_fallback(_use_fallback), default_int(_default_int),
          default_string(_default_string)
    {
        FswAssert(read(st, st_size));
    }
    SymbolTable::~SymbolTable() {}
    /**
     * Read symbols from a file and add them to the table.
     *
     * @note The table is not cleared by this function, so it can be
     * called on multiple files to add additional symbols.
     *
     * @param fname The data file.
     *
     * @return True on success.
     */
    bool SymbolTable::read(const std::string &fname)
    {
        if (dbverbose() >= 2)
            dbnprintf(100, "SymbolTable::read: %s\n", fname.c_str());
        FILE *in;
        FswAbortIfNot(fsw_fopen(in, fname, "r"), false);
        bool ret = read(in);
        fclose(in);
        return ret;
    }
    /**
     * Read symbols from a file and add them to the table.
     *
     * @note The table is not cleared by this function, so it can be
     * called on multiple files to add additional symbols.
     *
     * @param in The data file.
     *
     * @return True on success.
     */
    bool SymbolTable::read(FILE *in)
    {
        char wbuf[1024];
        char vbuf[1024];
        while (get_word(in, wbuf, sizeof(wbuf), true))
        {
            discard_whitespace(in);
            if (wbuf[0] == '#')
            {
                discard_line(in);
                continue;
            }
            get_word(in, vbuf, sizeof(vbuf));
            /*
             * string_to_uint is used as a backup here in the event
             * string_to_int fails.  See the documentation on raw_get
             * for more information.
             */
            int v = 0;
            uint val = 0;
            if (string_to_int(vbuf, v) == false)
            {
                FswAbortIfNot(string_to_uint(vbuf, val), false);
                v = static_cast<int>(val);
            }
            /*
             * Insert new entries.  If an entry already exists, it is
             * unchanged.
             */
            FswAbortIfNot(internal_add(wbuf, v), false);
            if (dbverbose() >= 2)
                dbnprintf(100, "\t%s: %d\n", wbuf, v);
        }
        return true;
    }
    /**
     * Read symbols from a string_table_entry and add them to the table.
     *
     * @note The table is not cleared by this function, so it can be
     * called on multiple files to add additional symbols.
     *
     * @param st The string_table_entry.
     * @param st_size The size of st.
     *
     * @return True on success.
     */
    bool SymbolTable::read(const string_table_entry *st, uint st_size)
    {
        for (uint i = 0; i < st_size; i++)
            FswAbortIfNot(internal_add(st[i].name, st[i].value), false);
        return true;
    }
    /**
     * Return true if this table does not have any entries.
     *
     * @return True if this table is empty.
     */
    bool SymbolTable::empty() const
    {
        return to_int_table.empty() && to_string_table.empty();
    }
    /**
     * Erase the results of any reads (including any reads that
     * happened in the 4 and 5 argument constructors ), and any
     * adds, leaving the SymbolTable ready for new reads.
     *
     * @return True on success.
     */
    bool SymbolTable::clear_reads()
    {
        to_int_table.clear();
        to_string_table.clear();
        return true;
    }
    /**
     * Copy all string<->int entries from another table, replacing any existing
     * entries that were in this table.
     *
     * @param other The table to copy from.
     *
     * @return True on success.
     */
    bool SymbolTable::copy_reads(const SymbolTable &other)
    {
        to_int_table = other.to_int_table;
        to_string_table = other.to_string_table;
        return true;
    }
    /**
     * Lookup a string and return its corresponding integer.
     *
     * The fallback mechanism is not used by this function.
     *
     * @param s The value to lookup.
     * @param i Returns the mapped value.
     *
     * @return True on success.  False if no mapping is found.  The
     * fallback mechanism is not used.
     */
    bool SymbolTable::raw_get(const std::string &s, int &i) const
    {
        return raw_get(s, i, false);
    }
    /**
     * Lookup a string and return its corresponding integer.  Note if the
     * the integer lookup fails, this function will perform an unsigned
     * integer lookup in an attempt to handle edge cases such as lookups
     * on values like 0x80000000 which aren't properly interpreted as a
     * negative value by string_to_int but are handled by string_to_uint
     * followed by a cast to int.
     *
     * @param s The value to lookup.
     * @param i Returns the mapped value.
     * @param _use_fallback True to use the fallback mechanism.
     *
     * @return True on success.  False if no mapping is found.
     */
    bool SymbolTable::raw_get(const std::string &s, int &i,
                              bool _use_fallback) const
    {
        str_int_m::const_iterator it = to_int_table.find(s);
        if (it != to_int_table.end())
        {
            i = it->second;
            return true;
        }
        if (_use_fallback)
        {
            uint val = 0;
            if (string_to_int(s.c_str(), i))
                return true;
            else if (string_to_uint(s.c_str(), val))
            {
                i = static_cast<int>(val);
                return true;
            }
        }
        return false;
    }
    /**
     * Lookup a string and return its corresponding (unsigned) integer.
     *
     * The fallback mechanism is not used by this function.
     *
     * @param s The value to lookup.
     * @param i Returns the mapped value.
     *
     * @return True on success.  False if no mapping is found.  The
     * fallback mechanism is not used.
     */
    bool SymbolTable::raw_get(const std::string &s, uint &i) const
    {
        int si = i;
        bool ret = raw_get(s, si, false);
        i = si;
        return ret;
    }
    /**
     * Lookup a string and return its corresponding (unsigned) integer.
     *
     * @param s The value to lookup.
     * @param i Returns the mapped value.
     * @param _use_fallback True to use the fallback mechanism.
     *
     * @return True on success.  False if no mapping is found.
     */
    bool SymbolTable::raw_get(const std::string &s, uint &i,
                              bool _use_fallback) const
    {
        int si = i;
        bool ret = raw_get(s, si, _use_fallback);
        i = si;
        return ret;
    }
    /**
     * Look up an integer and return its corresponding string.
     *
     * The fallback mechanism is not used by this function.
     *
     * @param i The value to look up.
     * @param s Returns the mapped value.
     *
     * @return True on success.  False is not mapping is found.  The
     * fallback mechanism is not used.
     */
    bool SymbolTable::raw_get(int i, std::string &s) const
    {
        return raw_get(i, s, false);
    }
    /**
     * Look up an integer and return its corresponding string.
     *
     * @param i The value to look up.
     * @param s Returns the mapped value.
     * @param _use_fallback True to use the fallback.
     *
     * @return True on success.  False is not mapping is found.
     */
    bool SymbolTable::raw_get(int i, std::string &s, bool _use_fallback) const
    {
        int_str_m::const_iterator it = to_string_table.find(i);
        if (it != to_string_table.end())
        {
            s = it->second;
            return true;
        }
        if (_use_fallback)
        {
            char buf[sizeof(int) * 3 + 1];
            if (snprintf(buf, sizeof(buf), "%d", i) >= 0)
            {
                s = buf;
                return true;
            }
        }
        return false;
    }
    /**
     * Map a string to an integer.
     *
     * Uses the fallback mechanism if enabled.
     *
     * @param s The value to map.
     *
     * @return The mapped value, the fallback value, or a default
     * value.
     */
    int SymbolTable::get(std::string s) const { return get(s, default_int); }
    /**
     * Map a string to an integer.
     *
     * Uses the fallback mechanism if enabled.
     *
     * @param s The value to map.
     *
     * @param def The default value if there is no mapping and the
     * fallback doesn't produce a value.
     *
     * @return The mapped value, the fallback value, or a default
     * value.
     */
    int SymbolTable::get(std::string s, int def) const
    {
        int i;
        if (raw_get(s, i, use_fallback))
            return i;
        return def;
    }
    /**
     * Map an integer to a string.
     *
     * Uses the fallback mechanism if enabled.
     *
     * @param i The value to map.
     *
     * @return The mapped value, the fallback value, or the default
     * value specified in the constructor.
     */
    std::string SymbolTable::get(int i) const { return get(i, default_string); }
    /**
     * Map an integer to a string.
     *
     * Uses the fallback mechanism if enabled.
     *
     * @param i The value to map.
     * @param def A default value to use if there is no mapped value
     * and if the fallback fails or isn't used.
     *
     * @return The mapped value.
     */
    std::string SymbolTable::get(int i, const std::string &def) const
    {
        std::string s;
        if (raw_get(i, s, use_fallback))
            return s;
        return def;
    }
    /**
     * Return true if this table contains an entry for the passed integer.
     *
     * @param i The value to query.
     *
     * @return True if this table contains the passed integer, i.e. `get(i)`
     *         will succeed.
     */
    bool SymbolTable::contains(int i) const
    {
        return to_string_table.count(i) > 0;
    }
    /**
     * Return true if this table contains an entry for the passed string.
     *
     * @param s The value to query.
     *
     * @return True if this table contains the passed string, i.e. `get(s)`
     *         will succeed.
     */
    bool SymbolTable::contains(const std::string &s) const
    {
        return to_int_table.count(s) > 0;
    }
    /**
     * Retrieve the mapping from string to integer values.
     *
     * @return A map containing the string -> integer mapping.
     */
    const str_int_m &SymbolTable::get_s2i_map() const { return to_int_table; }
    /**
     * Retrieve the mapping from integer to string values.
     *
     * @return A map containing the integer -> string mapping.
     */
    const int_str_m &SymbolTable::get_i2s_map() const
    {
        return to_string_table;
    }
    /**
     * Test if this SymbolTable is equal to another.
     *
     * @param other The other SymbolTable.
     *
     * @return True if equal.
     */
    bool SymbolTable::operator==(const SymbolTable &other) const
    {
        /*
         * No need to compare to_int_table since to_int_table should
         * be equal iff to_string_table is equal
         */
        return to_string_table == other.to_string_table &&
               default_string == other.default_string &&
               default_int == other.default_int &&
               use_fallback == other.use_fallback;
    }
    /**
     * Test if this SymbolTable is not equal to another.
     *
     * @param other The other SymbolTable.
     *
     * @return True if not equal.
     */
    bool SymbolTable::operator!=(const SymbolTable &other) const
    {
        return !(*this == other);
    }
    /**
     * Wrapper around add(). Sub-classes of SymbolTable may want to bail
     * out if add() fails during read(), but we do not.
     *
     * @param s The string.
     * @param i The integer.
     *
     * @return True, always. This ignores the return value of add().
     */
    bool SymbolTable::internal_add(const std::string &s, int i)
    {
        add(s, i);
        return true;
    }
    /**
     * Add a new mapping from a string to an integer and vice versa.
     *
     * Will not replace exiting mappings.
     *
     * @param s The string.
     * @param i The integer.
     *
     * @return True if the mapping in at least one direction was
     * added.  False neither mapping could be added (because they
     * already existed).
     */
    bool SymbolTable::add(const std::string &s, int i)
    {
        str_int_m::value_type si(s, i);
        std::pair<str_int_m::iterator, bool> ps = to_int_table.insert(si);
        int_str_m::value_type is(i, s);
        std::pair<int_str_m::iterator, bool> pi = to_string_table.insert(is);
        if (!ps.second && !pi.second)
            return false;
        return true;
    }
    /**
     * Print the output of the internal tables.
     *
     * @param name The name to print prior to the tables.
     *
     * @return True on success.
     */
    bool SymbolTable::dump(std::string name) const
    {
        dbnprintf(100, "%s:\n", name.c_str());
        dbnprintf(100, "    to_int_table:\n");
        str_int_m::const_iterator si;
        for (si = to_int_table.begin(); si != to_int_table.end(); si++)
        {
            dbnprintf(100, "\t\"%s\":\t%d\n", si->first.c_str(), si->second);
        }
        dbnprintf(100, "    to_string_table:\n");
        int_str_m::const_iterator ii;
        for (ii = to_string_table.begin(); ii != to_string_table.end(); ii++)
        {
            dbnprintf(100, "\t%d:\t\"%s\"\n", ii->first, ii->second.c_str());
        }
        return true;
    }
} /* end namespace Drone */
/* EOF */