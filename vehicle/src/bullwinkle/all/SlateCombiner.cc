/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#include "src/bullwinkle/all/SlateCombiner.h"

#include "src/bullwinkle/all/core/math/math_util.h"
#include "src/bullwinkle/all/slate_info.h"

#include <algorithm>
#include <limits>

namespace Drone
{
    /**
     * Constructor.
     */
    slate_combiner_in_t::slate_combiner_in_t(): builder(), name("unknown")
    {}

    /**
     * Constructor.
     *
     * @param builder Use this builder.
     * @param name Use this name.
     */
    slate_combiner_in_t::slate_combiner_in_t(SlateBuilder _builder,
                                             const std::string &_name):
        builder(_builder),
        name(_name)
    {}

    /**
     * Construct a slate combiner.
     */
    SlateCombiner::SlateCombiner():
        is_init(false),
        downselect(false),
        num_sources(0),
        stale_threshold(1U),
        bins(),
        sources(),
        combined_connected_tok(),
        slate()
    {}

    /**
     * Initialize a slate combiner. We watch the freshness elements for a
     * change to indicate that the data has been updated. These elements
     * must be of type INT64.
     *
     * @param configs Config file finder.
     * @param fresh Freshness element for the "a", "b", and "c" data.
     * @param builders_in List of input slate builders.
     * @param builder_out Builder for the destination slate.
     * @param config_key Config key for sharing configuration.
     * @param bind_outputs If true, bind to elements in \a builder_out instead
     * of creating them.
     * @param combiner_name The name of the combiner node. Defaults to
     * "combiner".
     *
     * @return True on success.
     */
    bool SlateCombiner::init(const Configs &configs,
                             const std::string &fresh,
                             const slate_combiner_in_v &builders_in,
                             SlateBuilder builder_out,
                             const std::string &config_key,
                             const bool bind_outputs,
                             const std::string &combiner_name)
    {
        SacAbortIf(is_init, false);

        sharer_config_v config_list;
        config_list.push_back(sharer_config_t(config_key, ""));
        SacAbortIfNot(init(configs,
                           fresh,
                           builders_in,
                           builder_out,
                           config_list,
                           bind_outputs,
                           combiner_name),
                      false);

        return true;
    }

    /**
     * Initialize a slate combiner. We watch the freshness elements for a
     * change to indicate that the data has been updated. These elements
     * must be of type INT64.
     *
     * @param configs Config file finder.
     * @param fresh Freshness element for the "a", "b", and "c" data.
     * @param builders_in List of input slate builders.
     * @param builder_out Builder for the destination slate.
     * @param config_list The list of configs (either a config key or a
     *                    str_v_v) specifying which devices to share and the
     *                    corresponding prefix (which may be empty) to prepend
     *                    to the shared device names.
     * @param bind_outputs If true, bind to elements in \a builder_out instead
     *                     of creating them.
     * @param combiner_name The name of the combiner node. Defaults to
     *                      "combiner".
     * @param access The permissions of the combiner elements (non-metadata)
     *               when they are created. Defaults to slate_read_only.
     * @param _downselect Whether to use "downselect" mode when combining
     *                    instead of the default per-element median. This can be
     *                    used to avoid "tearing" messages that require inter-
     *                    element coherency. If there is not a clear majority
     *                    (two or more exactly equal sources), then the first
     *                    connected source will always be used.
     * @param bind_metrics Whether to bind to metrics devices instead of
     *                     creating them.
     * @param _stale_threshold Number of cycles after which a source will no
     *                         longer be considered "fresh" if it has not
     *                         updated. Must be in the range [1, 255]. Defaults
     *                         to 1.
     *
     * @return True on success.
     */
    bool SlateCombiner::init(const Configs &configs,
                             const std::string &fresh,
                             const slate_combiner_in_v &builders_in,
                             SlateBuilder builder_out,
                             const sharer_config_v &config_list,
                             const bool bind_outputs,
                             const std::string &combiner_name,
                             const slate_elem_access_t access,
                             const bool _downselect,
                             const bool bind_metrics,
                             const UINT8 _stale_threshold)
    {
        SacAbortIf(is_init, false);
        SacAbortIf(builders_in.empty(), false);

        /*
         * Check size.
         */
        if (SacOutsideRange(builders_in.size(), 1U, num_sources_max + 1))
        {
            SacPrefix();
            dbnprintf(200,
                      ": The number of combiner input source is limited "
                      "to %zu.\n",
                      num_sources_max);
            return false;
        }

        num_sources = builders_in.size();

        /*
         * We keep a mapping of type_string->bin_index. This allows us to
         * bin all elements by their type and batch-process them at runtime.
         */
        str_size_t_m bin_select;

        bool initialization_ok = true;

        /*
         * Parse each configuration and build up a descriptor table with the
         * element IDs and a function to handle each element.
         */
        for (size_t i = 0; i < config_list.size(); ++i)
        {
            const sharer_config_t &entry = config_list[i];

            str_v_v lines;
            SacAbortIfNot(entry.read_config_str_v_v(configs, lines), false);

            for (size_t j = 0; j < lines.size(); j++)
            {
                const str_v &line = lines[j];

                /*
                 * Make sure there are at least two fields. More than two can be
                 * present if we're using scaling or enum information.
                 */
                if (SacIfNot(line.size() >= 2))
                {
                    SacPrefix();
                    dbnprintf(200,
                              ": Error parsing line (need path, type):\n%s\n",
                              join(line).c_str());
                    return false;
                }

                /*
                 * Skip ignored elements.
                 */
                if (line[0] == sharer_ignore_element_name)
                {
                    continue;
                }

                const std::string path = entry.element_prefix + line[0];
                std::string type = line[1];

                /*
                 * packed_bools are bitpacked in the network packet, but they're
                 * the same as regular bools once they're in slate.
                 */
                if (type == "packed_bool")
                {
                    type = "bool";
                }

                /*
                 * Select the appropriate bin. If it doesn't exist,
                 * create it.
                 */
                const std::pair<std::string, size_t> val =
                    make_pair(type, bins.size());

                const std::pair<str_size_t_m::iterator, bool> ret =
                    bin_select.insert(val);

                /*
                 * Create a bin if it didn't exist.
                 */
                if (ret.second)
                {
                    bins.resize(bins.size() + 1);
                }

                element_bin_t &bin = bins[ret.first->second];

                if (type == "bool")
                {
                    initialization_ok = init_element<bool>(builders_in,
                                                           builder_out,
                                                           path,
                                                           bind_outputs,
                                                           bin,
                                                           access) &&
                                        initialization_ok;
                }
                else if (type == "int8")
                {
                    initialization_ok = init_element<INT8>(builders_in,
                                                           builder_out,
                                                           path,
                                                           bind_outputs,
                                                           bin,
                                                           access) &&
                                        initialization_ok;
                }
                else if (type == "int16")
                {
                    initialization_ok = init_element<INT16>(builders_in,
                                                            builder_out,
                                                            path,
                                                            bind_outputs,
                                                            bin,
                                                            access) &&
                                        initialization_ok;
                }
                else if (type == "int32")
                {
                    initialization_ok = init_element<INT32>(builders_in,
                                                            builder_out,
                                                            path,
                                                            bind_outputs,
                                                            bin,
                                                            access) &&
                                        initialization_ok;
                }
                else if (type == "int64")
                {
                    initialization_ok = init_element<INT64>(builders_in,
                                                            builder_out,
                                                            path,
                                                            bind_outputs,
                                                            bin,
                                                            access) &&
                                        initialization_ok;
                }
                else if (type == "uint8")
                {
                    initialization_ok = init_element<UINT8>(builders_in,
                                                            builder_out,
                                                            path,
                                                            bind_outputs,
                                                            bin,
                                                            access) &&
                                        initialization_ok;
                }
                else if (type == "uint16")
                {
                    initialization_ok = init_element<UINT16>(builders_in,
                                                             builder_out,
                                                             path,
                                                             bind_outputs,
                                                             bin,
                                                             access) &&
                                        initialization_ok;
                }
                else if (type == "uint32")
                {
                    initialization_ok = init_element<UINT32>(builders_in,
                                                             builder_out,
                                                             path,
                                                             bind_outputs,
                                                             bin,
                                                             access) &&
                                        initialization_ok;
                }
                else if (type == "uint64")
                {
                    initialization_ok = init_element<UINT64>(builders_in,
                                                             builder_out,
                                                             path,
                                                             bind_outputs,
                                                             bin,
                                                             access) &&
                                        initialization_ok;
                }
                else if (type == "float")
                {
                    initialization_ok = init_element<float>(builders_in,
                                                            builder_out,
                                                            path,
                                                            bind_outputs,
                                                            bin,
                                                            access) &&
                                        initialization_ok;
                }
                else if (type == "double")
                {
                    initialization_ok = init_element<double>(builders_in,
                                                             builder_out,
                                                             path,
                                                             bind_outputs,
                                                             bin,
                                                             access) &&
                                        initialization_ok;
                }
                else
                {
                    SacPrefix();
                    dbnprintf(200, ": Unsupported type '%s'\n", type.c_str());
                    initialization_ok = false;
                }
            }
        }

        SacAbortIfNot(initialization_ok, false);

        /*
         * A stale threshold of zero is nonsensical: if a source could be marked
         * stale after zero cycles without an update, it could never be fresh.
         */
        SacAbortIf(_stale_threshold == 0U, false);
        stale_threshold = _stale_threshold;

        /*
         * Ensure all slates are peers so we can just store the one.
         */
        for (size_t i = 0; i < builders_in.size(); ++i)
        {
            SacAbortIfNot(builders_in[i].builder.is_peer(builder_out), false);
        }

        slate = builder_out.slate(slate_no_validation);

        SlateBuilder sub = builder_out.sub_slate(combiner_name);

        /*
         * Create all the source metrics.
         */
        for (size_t i = 0; i < builders_in.size(); ++i)
        {
            const slate_combiner_in_t &in = builders_in[i];

            source_t source;
            SacAbortIfNot(
                source.init(in.builder, in.name, fresh, stale_threshold, sub),
                false);
            sources.push_back(source);
        }

        /*
         * Bind or create the combined source metrics to indicate the status of
         * the SlateCombiner output.
         */
        if (bind_metrics)
        {
            SacAbortIfNot(sub.bind("connected", combined_connected_tok), false);
        }
        else
        {
            SacAbortIfNot(sub.create("connected",
                                     false,
                                     shard_sync,
                                     slate_read_only,
                                     combined_connected_tok),
                          false);
        }

        /**
         * Save off whether we're going to be using downselect mode.
         */
        downselect = _downselect;

        is_init = true;
        return true;
    }

    /**
     * Process all the elements and apply the operations to them. This method
     * accepts sources with any sequence number.
     *
     * @return True on success.
     */
    bool SlateCombiner::combine() RUNTIME
    {
        SacAbortIfNot(is_init, false);
        SacAbortIfNot(validate_and_combine(sharer_invalid_sequence_number),
                      false);
        return true;
    }

    /**
     * Process all the elements and apply the operations to them.
     *
     * @param accepted_sequence_number Sources will only be considered "fresh"
     *                                 if their freshness elements exactly equal
     *                                 this value.
     *
     * @return True on success.
     */
    bool SlateCombiner::validate_and_combine(
        const nano_t accepted_sequence_number) RUNTIME
    {
        SacAbortIfNot(is_init, false);

        /*
         * Run through all the sources, compute connectedness and build the
         * freshness indices.
         */
        const size_t sources_size = sources.size();
        SacAssert(sources_size <= num_sources_max);
        size_t indices[num_sources_max];
        size_t indices_len = 0;

        bool &combined_connected = slate.load(combined_connected_tok);
        combined_connected = false;

        for (size_t i = 0; i < sources_size; ++i)
        {
            const source_t &source = sources[i];

            const INT64 fresh = slate.load(source.fresh_tok);
            INT64 &fresh_last = slate.load(source.fresh_last_tok);
            UINT8 &fresh_age = slate.load(source.fresh_age_tok);
            bool &connected = slate.load(source.connected_tok);
            bool updated_this_cycle = false;
            connected = false;

            /*
             * In all cases, only consider the source to have updated if the
             * fresh element is neither zero (to defend against default-
             * initialization cases) nor the minimum possible value (to defend
             * against "proxy sender" cases).
             *
             * After meeting those criteria, if exact sequence number matching
             * was requested, then the source is fresh if it matches; otherwise,
             * it is fresh if it has changed since last time.
             */
            constexpr INT64 seq_default = std::numeric_limits<INT64>::min();
            if (fresh != 0 && fresh != seq_default)
            {
                if (accepted_sequence_number == sharer_invalid_sequence_number)
                {
                    updated_this_cycle = (fresh != fresh_last);
                }
                else
                {
                    updated_this_cycle = (fresh == accepted_sequence_number);
                }
            }

            /*
             * If the source updated on this cycle, remember its freshness value
             * and reset its age to zero. Otherwise, increment the age, but
             * clamp at the threshold.
             */
            if (updated_this_cycle)
            {
                fresh_last = fresh;
                fresh_age = 0U;
            }
            else if (fresh_age < stale_threshold)
            {
                ++fresh_age;
            }

            /*
             * If the age of the last update is below the staleness threshold,
             * the source is considered "fresh" (or connected), and its inputs
             * are eligible for combining.
             */
            if (fresh_age < stale_threshold)
            {
                connected = true;
                combined_connected = true;
                indices[indices_len++] = i + 1;
            }
        }

        /*
         * If we are using downselect mode, truncate the indices list down to
         * just one source, which will force us to simply copy its data
         * wholesale, instead of performing a per-element median.
         */
        if (downselect)
        {
            switch (indices_len)
            {
            case 1:
            case 2:
            {
                /*
                 * If there are only one or two sources connected, always pick
                 * the first one: if their inputs are the same, it doesn't
                 * matter which we pick; and if they are different, we have to
                 * pick one arbitrarily, anyway.
                 */
                indices_len = 1;
                break;
            }
            case 3:
            {
                /*
                 * If there are three sources connected (call them A, B, C),
                 * always pick A, unless B and C's inputs are fully identical:
                 *
                 *  - If all three are identical, it doesn't matter which we
                 *    pick, by definition. (But this method will pick B.)
                 *  - If A and B agree, but C doesn't, we'll pick A.
                 *  - If A and C agree, but B doesn't, we'll pick A.
                 *  - If B and C agree, but A doesn't, we'll pick B.
                 *  - If all three disagree, we'll pick A. This is fine, since
                 *    we lack a clear winner and have to make an arbitrary
                 *    choice.
                 */
                static constexpr size_t a_idx = 1U;
                static constexpr size_t b_idx = 2U;
                static constexpr size_t c_idx = 3U;

                bool b_eq_c = true;
                for (const auto &bin : bins)
                {
                    SacAssert(bin.eq_fn);
                    SacAssert((this->*bin.eq_fn)(b_idx, c_idx, bin, b_eq_c));
                }

                indices[0] = b_eq_c ? b_idx : a_idx;
                indices_len = 1;
                break;
            }
            default:
            {
                /*
                 * No inputs connected - nothing to do.
                 */
                static_assert(num_sources_max == 3);
                break;
            }
            }
        }

        /*
         * Dispatch the right handler for each entry depending on how many
         * sources are connected.
         */
        switch (indices_len)
        {
        case 1:
        {
            for (size_t i = 0; i < bins.size(); ++i)
            {
                const element_bin_t &bin = bins[i];
                SacAssert(bin.copy_fn);
                SacAssert((this->*bin.copy_fn)(indices[0], bin));
            }
            break;
        }

        case 2:
        {
            for (size_t i = 0; i < bins.size(); ++i)
            {
                const element_bin_t &bin = bins[i];
                SacAssert(bin.max_fn);
                SacAssert((this->*bin.max_fn)(indices[0], indices[1], bin));
            }
            break;
        }

        case 3:
        {
            for (size_t i = 0; i < bins.size(); ++i)
            {
                const element_bin_t &bin = bins[i];
                SacAssert(bin.median_fn);
                SacAssert((this->*bin.median_fn)(bin));
            }
            break;
        }

        default:
        {
            /*
             * No inputs connected - don't update outputs.
             */
            static_assert(num_sources_max == 3);
            break;
        }
        }

        return true;
    }

    /**
     * Initialize freshness metrics for a source.
     *
     * @param source_builder Source SlateBuilder.
     * @param source_name Name of the source.
     * @param fresh Path of freshness element.
     * @param _stale_threshold Number of cycles after which a source will no
     *                         longer be considered "fresh".
     * @param combiner_builder Subslate for the combiner state.
     *
     * @return True on success.
     */
    bool SlateCombiner::source_t::init(const SlateBuilder &source_builder,
                                       const std::string &source_name,
                                       const std::string &fresh,
                                       const UINT8 _stale_threshold,
                                       SlateBuilder &combiner_builder)
    {
        const INT64 seq_default = std::numeric_limits<INT64>::min();

        SacAbortIfNot(source_builder.bind(fresh, fresh_tok), false);
        SacAbortIfNot(combiner_builder.create(source_name + ".fresh_last",
                                              seq_default,
                                              shard_sync,
                                              fresh_last_tok),
                      false);
        SacAbortIfNot(combiner_builder.create(source_name + ".fresh_age",
                                              _stale_threshold,
                                              shard_sync,
                                              fresh_age_tok),
                      false);
        SacAbortIfNot(combiner_builder.create(source_name + ".connected",
                                              false,
                                              shard_sync,
                                              slate_read_only,
                                              connected_tok),
                      false);

        return true;
    }

    /**
     * If the input elements have an enum registered, verify consistency and
     * ensure the output element has the same enum registered.
     *
     * @param builders_in List of input SlateBuilders.
     * @param builder_out Builder for the destination slate.
     * @param path Path of the slate element to operate on.
     * @param bind_outputs Whether to verify the output element enum or register
     *                     it since we just created the output element.
     *
     * @return True on success.
     */
    bool
    SlateCombiner::register_output_enum(const slate_combiner_in_v &builders_in,
                                        SlateBuilder builder_out,
                                        const std::string &path,
                                        const bool bind_outputs)
    {
        /*
         * Get the first input slate builder, and use it as the comparison
         * reference for consistency checks and output enum registration.
         */
        SacAbortIfNotOpUint(builders_in.size(), >=, 1, false);
        const slate_combiner_in_t &slate_combiner_in_first = builders_in[0];
        const SlateBuilder &builder_first = slate_combiner_in_first.builder;
        const std::string input_path_first =
            slate_join_path(builder_first.get_subtree_path(), path);

        /*
         * Get the enum definition from the first input slate builder,
         * if an enum registry is present.
         */
        const bool has_enum_registry_first = builder_first.has_enum_registry();
        bool enum_exists_first = false;
        std::string enum_name_first;
        SymbolTable symbol_table_first;
        if (has_enum_registry_first)
        {
            SacAbortIfNot(builder_first.get_element_enum(path,
                                                         enum_exists_first,
                                                         enum_name_first,
                                                         symbol_table_first),
                          false);
        }

        bool input_builders_ok = true;

        /*
         * Iterate through the rest of the input slate builders,
         * check enum registry presence, and check enum consistency.
         */
        for (size_t i = 1; i < builders_in.size(); ++i)
        {
            const slate_combiner_in_t &slate_combiner_in = builders_in[i];
            const SlateBuilder &builder = slate_combiner_in.builder;
            const std::string input_path =
                slate_join_path(builder.get_subtree_path(), path);
            const bool has_enum_registry = builder.has_enum_registry();

            if (has_enum_registry_first != has_enum_registry)
            {
                SacPrefix();
                dbnprintf(
                    512,
                    ": Either all input slate builders must be associated with "
                    "an enum registry, or none can be:\n"
                    "'%s' %s a registry, but '%s' %s.\n",
                    slate_combiner_in_first.name.c_str(),
                    has_enum_registry_first ? "has" : "does not have",
                    slate_combiner_in.name.c_str(),
                    has_enum_registry ? "does" : "does not");
                input_builders_ok = false;
                continue;
            }

            if (has_enum_registry)
            {
                bool enum_exists = false;
                std::string enum_name;
                SymbolTable symbol_table;

                SacAbortIfNot(builder.get_element_enum(
                                  path, enum_exists, enum_name, symbol_table),
                              false);

                if (enum_exists_first != enum_exists)
                {
                    SacPrefix();
                    dbnprintf(
                        512,
                        ": Input '%s' %s an enumerated value, but '%s' %s.\n",
                        input_path_first.c_str(),
                        enum_exists_first ? "is" : "is not",
                        input_path.c_str(),
                        enum_exists ? "is" : "is not");
                    input_builders_ok = false;
                    continue;
                }

                if (symbol_table_first != symbol_table)
                {
                    SacPrefix();
                    dbnprintf(512,
                              ": Inputs '%s' and '%s' have "
                              "inconsistent symbol tables:\n",
                              input_path_first.c_str(),
                              input_path.c_str());
                    symbol_table_first.dump();
                    symbol_table.dump();
                    input_builders_ok = false;
                    continue;
                }
            }
        }

        SacAbortIfNot(input_builders_ok, false);

        const std::string builder_path_out = builder_out.get_subtree_path();
        const std::string output_path = slate_join_path(builder_path_out, path);
        const bool has_enum_registry_out = builder_out.has_enum_registry();

        if (bind_outputs)
        {
            /*
             * Check output builder has an enum registry if inputs do.
             */
            SacMsgAbortIfNot(has_enum_registry_first == has_enum_registry_out,
                             false,
                             512,
                             "Cannot check consistency: input '%s' %s a "
                             "enum registry, but output '%s' %s.",
                             slate_combiner_in_first.name.c_str(),
                             has_enum_registry_first ? "has" : "does not have",
                             builder_path_out.c_str(),
                             has_enum_registry_out ? "does" : "does not");

            /*
             * Check enum consistency between output and inputs.
             */
            if (has_enum_registry_out)
            {
                bool enum_exists_out = false;
                std::string enum_name_out;
                SymbolTable symbol_table_out;

                SacAbortIfNot(builder_out.get_element_enum(path,
                                                           enum_exists_out,
                                                           enum_name_out,
                                                           symbol_table_out),
                              false);

                SacMsgAbortIfNot(
                    enum_exists_first == enum_exists_out,
                    false,
                    512,
                    "Input '%s' %s an enumerated value, but output '%s' %s.\n",
                    input_path_first.c_str(),
                    enum_exists_first ? "is" : "is not",
                    output_path.c_str(),
                    enum_exists_out ? "is" : "is not");

                if (symbol_table_first != symbol_table_out)
                {
                    SacPrefix();
                    dbnprintf(512,
                              ": Input '%s' and ouput '%s' have "
                              "inconsistent symbol tables:\n",
                              input_path_first.c_str(),
                              output_path.c_str());
                    symbol_table_first.dump();
                    symbol_table_out.dump();
                    return false;
                }
            }
        }
        else
        {
            /*
             * If an enum exists for the input element,
             * attempt to register it for the output element.
             */
            if (enum_exists_first)
            {
                SacMsgAbortIfNot(has_enum_registry_out,
                                 false,
                                 256,
                                 "Cannot create enumerated element '%s': "
                                 "builder doesn't have enum registry.",
                                 output_path.c_str());
                SacAbortIfNot(builder_out.register_enum(path,
                                                        enum_name_first,
                                                        symbol_table_first),
                              false);
            }
        }

        return true;
    }

    /**
     * Initialize an element descriptor entry.
     *
     * @param        builders_in List of input slate builders.
     * @param        builder_out Builder for the destination slate.
     * @param        path The path to the element.
     * @param        bind_outputs If true, bind to elements in \a builder_out
     *               instead of creating them.
     * @param[inout] bin Appends to this bin.
     * @param        access The permissions of the elements that are created.
     *
     * @return True on success.
     */
    template <typename T>
    bool SlateCombiner::init_element(const slate_combiner_in_v &builders_in,
                                     SlateBuilder builder_out,
                                     const std::string &path,
                                     const bool bind_outputs,
                                     element_bin_t &bin,
                                     const slate_elem_access_t access)
    {
        SacAbortIf(is_init, false);

        const std::string output_path =
            slate_join_path(builder_out.get_subtree_path(), path);

        slate_element_t id_out;
        if (bind_outputs)
        {
            SacMsgAbortIfNot(builder_out.get_element_id<T>(path, id_out),
                             false,
                             200,
                             "Cannot bind to element '%s'",
                             output_path.c_str());

            /*
             * Verify that the slate element is writable.
             */
            SacMsgAbortIfNot(slate_id_can_write(id_out),
                             false,
                             200,
                             "Element '%s' is read only",
                             output_path.c_str());
        }
        else
        {
            SacMsgAbortIfNot(builder_out.create_element<T>(
                                 path, T(), shard_sync, access, id_out),
                             false,
                             200,
                             "Cannot create element '%s'",
                             output_path.c_str());
        }

        bin.ids.push_back(id_out);

        SacAbortIfNot(
            register_output_enum(builders_in, builder_out, path, bind_outputs),
            false);

        for (const auto &slate_combiner_in : builders_in)
        {
            const SlateBuilder &builder = slate_combiner_in.builder;
            const std::string input_path =
                slate_join_path(builder.get_subtree_path(), path);

            slate_element_t id = slate_element_default;
            SacMsgAbortIfNot(builder.get_element_id<T>(path, id),
                             false,
                             200,
                             "Cannot bind to element '%s'",
                             input_path.c_str());

            bin.ids.push_back(id);
        }

        /*
         * Wipe over the functions every time - this doesn't hurt and is the
         * easiest place to do it since we have T here.
         */
        bin.eq_fn = &SlateCombiner::bin_eq<T>;
        bin.copy_fn = &SlateCombiner::bin_copy<T>;
        bin.max_fn = &SlateCombiner::bin_max<T>;
        bin.median_fn = &SlateCombiner::bin_median<T>;

        return true;
    }

    /**
     * Determine whether all values are equal for each element.
     *
     * @tparam T The type of the elements.
     *
     * @param src1 The 1-based index of the first source to compare. This index
     *             is 1-based because index 0 is the output element.
     * @param src2 The 1-based index of the second source to compare. This index
     *             is 1-based because index 0 is the output element.
     * @param bin Process all elements in this bin.
     * @param[in,out] result If any of the elements in this bin are not exactly
     *                       equal between the two sources, this boolean will be
     *                       set to false.
     *
     * @return True on success.
     */
    template <typename T>
    bool SlateCombiner::bin_eq(size_t src1,
                               size_t src2,
                               const element_bin_t &bin,
                               bool &result) RUNTIME
    {
        SacAbortIfNot(is_init, false);

        const size_t ids_size = bin.ids.size();
        const size_t stride = num_sources + 1;

        for (size_t i = 0; i < ids_size; i += stride)
        {
            result &= (slate.load<T>(bin.ids[i + src1]) ==
                       slate.load<T>(bin.ids[i + src2]));
        }

        return true;
    }

    /**
     * Copy values from one source to the output for each element.
     *
     * @tparam T The type of the elements.
     *
     * @param src The 1-based index of the valid source. This index is 1-based
     *        because index 0 is the output element.
     * @param bin Process all elements in this bin.
     *
     * @return True on success.
     */
    template <typename T>
    bool SlateCombiner::bin_copy(size_t src, const element_bin_t &bin) RUNTIME
    {
        SacAbortIfNot(is_init, false);

        const size_t ids_size = bin.ids.size();
        const size_t stride = num_sources + 1;
        bool success = true;

        /*
         * Process each element. Accumulate the success into a bool using &=
         * so that we can report success at the end of the function without
         * branching inside the loop, and without short-circuiting and skipping
         * later stores if an earlier store fails. Note that store() should
         * never fail because we check that it's possible write to all outputs
         * in init_element(), but report just in case.
         */
        for (size_t i = 0; i < ids_size; i += stride)
        {
            const T val = slate.load<T>(bin.ids[i + src]);
            success &= slate.store<T>(bin.ids[i], val);
        }

        return success;
    }

    /**
     * Pick the maximum of two values for each element.
     *
     * @tparam T The type of the elements.
     *
     * @param src1 The 1-based index of the first source. This index is 1-based
     *        because index 0 is the output element.
     * @param src2 The 1-based index of the second source. This index is 1-based
     *        because index 0 is the output element.
     * @param bin Process all elements in this bin.
     *
     * @return True on success.
     */
    template <typename T>
    bool SlateCombiner::bin_max(size_t src1,
                                size_t src2,
                                const element_bin_t &bin) RUNTIME
    {
        SacAbortIfNot(is_init, false);

        const size_t ids_size = bin.ids.size();
        const size_t stride = num_sources + 1;
        bool success = true;

        /*
         * Process each element. Accumulate the success into a bool using &=
         * so that we can report success at the end of the function without
         * branching inside the loop, and without short-circuiting and skipping
         * later stores if an earlier store fails. Note that store() should
         * never fail because we check that it's possible write to all outputs
         * in init_element(), but report just in case.
         */
        for (size_t i = 0; i < ids_size; i += stride)
        {
            const T val1 = slate.load<T>(bin.ids[i + src1]);
            const T val2 = slate.load<T>(bin.ids[i + src2]);

            const T maxval = std::max(val1, val2);

            success &= slate.store<T>(bin.ids[i], maxval);
        }

        return success;
    }

    /**
     * Perform a median of three values for each element.
     *
     * @tparam T The type of the elements that will be medianed.
     *
     * @param bin Process all elements in this bin.
     *
     * @return True on success.
     */
    template <typename T>
    bool SlateCombiner::bin_median(const element_bin_t &bin) RUNTIME
    {
        SacAbortIfNot(is_init, false);
        static_assert(num_sources_max == 3);
        SacAbortIfNot(num_sources == 3, false);

        const size_t ids_size = bin.ids.size();
        const size_t stride = num_sources_max + 1;
        bool success = true;

        /*
         * Process each element. Accumulate the success into a bool using &=
         * so that we can report success at the end of the function without
         * branching inside the loop, and without short-circuiting and skipping
         * later stores if an earlier store fails. Note that store() should
         * never fail because we check that it's possible write to all outputs
         * in init_element(), but report just in case.
         */
        for (size_t i = 0; i < ids_size; i += stride)
        {
            const T val1 = slate.load<T>(bin.ids[i + 1]);
            const T val2 = slate.load<T>(bin.ids[i + 2]);
            const T val3 = slate.load<T>(bin.ids[i + 3]);

            const T median = median3(val1, val2, val3);

            success &= slate.store<T>(bin.ids[i], median);
        }

        return success;
    }

    /**
     * Default-construct an element descriptor.
     */
    SlateCombiner::element_bin_t::element_bin_t():
        ids(),
        eq_fn(),
        copy_fn(),
        max_fn(),
        median_fn()
    {}

} /* end namespace Drone */