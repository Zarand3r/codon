/**
 * @author Richard Bao
 *
 * Slate path utilities. Element paths are hierarchical, dot-separated names
 * (e.g. "nav.altitude_m", "heap.bytes"). These helpers validate a path, join a
 * subtree with a relative path, and compute a path relative to a subtree — used
 * by SlateLayout (element registration) and SlateBuilder (sub-slate scoping,
 * masked-path checks).
 *
 * Cold path: paths are resolved at Slate *build* time only (a runtime load/store
 * goes by packed id, never by string).
 */

#ifndef SLATE_PATH_H
#define SLATE_PATH_H

#include "src/bullwinkle/all/core/fsw.h"

#include <string>
#include <string_view>

namespace Drone
{
    /** The separator between path segments. */
    static const char slate_path_sep = '.';

    /**
     * A path is valid if it is non-empty, contains only [A-Za-z0-9_] segment
     * characters separated by single '.'s, and has no empty segment (no leading,
     * trailing, or doubled separator).
     */
    inline bool validate_slate_path(const std::string_view path)
    {
        if (path.empty())
        {
            return false;
        }
        bool segment_empty = true; // current segment has no chars yet
        for (const char c : path)
        {
            if (c == slate_path_sep)
            {
                if (segment_empty)
                {
                    return false; // leading or doubled separator
                }
                segment_empty = true;
                continue;
            }
            const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                            (c >= '0' && c <= '9') || c == '_';
            if (!ok)
            {
                return false;
            }
            segment_empty = false;
        }
        return !segment_empty; // reject a trailing separator
    }

    /**
     * Join a subtree path and a relative path with the separator. An empty
     * operand is dropped (joining with the root subtree "" yields the relative
     * path unchanged).
     */
    inline std::string slate_join_path(const std::string_view subtree,
                                       const std::string_view relative)
    {
        if (subtree.empty())
        {
            return std::string(relative);
        }
        if (relative.empty())
        {
            return std::string(subtree);
        }
        std::string out;
        out.reserve(subtree.size() + 1 + relative.size());
        out.append(subtree);
        out.push_back(slate_path_sep);
        out.append(relative);
        return out;
    }

    /**
     * Compute `full` relative to `base`: true iff `full` is within `base`'s
     * subtree (base is empty, base == full, or full begins with "base."). On
     * success `out` receives the remainder after `base` (empty if full == base,
     * the whole of full if base is empty).
     */
    inline bool slate_rel_path(const std::string_view base,
                               const std::string_view full,
                               std::string_view &out)
    {
        if (base.empty())
        {
            out = full;
            return true;
        }
        if (full == base)
        {
            out = std::string_view();
            return true;
        }
        if (full.size() > base.size() && full.substr(0, base.size()) == base &&
            full[base.size()] == slate_path_sep)
        {
            out = full.substr(base.size() + 1);
            return true;
        }
        return false;
    }

    /** std::string out overload (some callers store the remainder as a string). */
    inline bool slate_rel_path(const std::string_view base,
                               const std::string_view full, std::string &out)
    {
        std::string_view sv;
        if (!slate_rel_path(base, full, sv))
        {
            return false;
        }
        out = std::string(sv);
        return true;
    }

} /* end namespace Drone */

#endif /* SLATE_PATH_H */
