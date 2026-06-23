/**
 * @author Stefan Moluf
 * @date   12/18/09
 */
#include "src/bullwinkle/all/TelemetryRelay.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/core/str_util.h"
#include "src/bullwinkle/all/io/AnyDgramConnection.h"
#include "src/bullwinkle/all/io/BwpChannelWriter.h"
#include "src/bullwinkle/all/service.h"
#include "src/bullwinkle/all/telem_group_t.enum.h"
namespace Drone
{
    namespace
    {
        /**
         * Parses only the portion of the telemetry configs relevant to control
         * code. See RuntimeFlowInfoParser for a complete parse.
         */
        class ControlFlowInfoParser
        {
        public:
            /**
             * Destructor.
             */
            virtual ~ControlFlowInfoParser() {}
            /**
             * Parse a configuration file.
             *
             * @param file_lines Lines of the config file.
             * @param muxed_groups A set of groups that should have the muxing
             *                     flag added.
             * @param[out] flow_info Returns list of TelemetryFlowInfo objects.
             *
             * @return True on success.
             */
            bool parse_file(const str_v_v &file_lines,
                            const std::set<telem_group_t> &muxed_groups,
                            std::vector<TelemetryFlowInfo> &flow_info)
            {
                size_t line = 0;
                /*
                 * Start off each new telemetry flow definition.
                 */
                while (line < file_lines.size())
                {
                    /*
                     * There must be at least three lines to form a valid
                     * telemetry flow definition.
                     */
                    if (line + 2 >= file_lines.size())
                    {
                        FswPrefix();
                        dbstring(": Incomplete telemetry flow definition.\n");
                        return false;
                    }
                    TelemetryFlowInfo info;
                    /*
                     * Get header line.
                     */
                    const str_v &header_line = file_lines[line++];
                    FswAbortIfNot(parse_header_line(header_line, info), false);
                    /*
                     * Get type line.
                     */
                    const str_v &type_line = file_lines[line++];
                    FswAbortIfNot(parse_type_line(type_line, info), false);
                    /*
                     * Get destination.
                     */
                    const str_v &dest_line = file_lines[line++];
                    FswAbortIfNot(parse_destination_line(dest_line, info),
                                  false);
                    while (line < file_lines.size())
                    {
                        const str_v &buf_line = file_lines[line];
                        FswAbortIfNot(buf_line.size() > 0, false);
                        if (buf_line[0] == "bandwidth")
                        {
                            FswAbortIfNot(parse_bandwidth_line(buf_line, info),
                                          false);
                        }
                        else if (buf_line[0] == "idle_timeout")
                        {
                            FswAbortIfNot(parse_timeout_line(buf_line, info),
                                          false);
                        }
                        else if (buf_line[0] == "store_and_forward")
                        {
                            FswAbortIfNot(parse_sf_line(buf_line, info), false);
                        }
                        else if (buf_line[0] == "external_record")
                        {
                            FswAbortIfNot(parse_er_line(buf_line, info), false);
                        }
                        else if (buf_line[0] == "buffer")
                        {
                            FswAbortIfNot(parse_buffer_line(buf_line, info),
                                          false);
                        }
                        else if (buf_line[0] == "name")
                        {
                            FswAbortIfNot(parse_name_line(buf_line, info),
                                          false);
                        }
                        else if (buf_line[0] == "disabled_destination")
                        {
                            FswAbortIfNot(
                                parse_disabled_destination_line(buf_line, info),
                                false);
                        }
                        else if (buf_line[0].compare(0,
                                                     strlen("alt_destination_"),
                                                     "alt_destination_") == 0)
                        {
                            FswAbortIfNot(
                                parse_alt_destination_lines(buf_line, info),
                                false);
                        }
                        else if (buf_line[0] == "override_source_role_inst")
                        {
                            FswAbortIfNot(
                                parse_override_source_role_inst(buf_line, info),
                                false);
                        }
                        else if (buf_line[0] == "precomputed_annotation_file")
                        {
                            FswAbortIfNeq(info.type, telem_annotated_dgram,
                                          false);
                            FswAbortIfNot(parse_precomputed_annotation_file(
                                              buf_line, info),
                                          false);
                        }
                        else if (buf_line[0] == "timestamp_channel")
                        {
                            FswAbortIfNeq(info.type, telem_annotated_dgram,
                                          false);
                            FswAbortIfNot(
                                parse_timestamp_channel(buf_line, info), false);
                        }
                        else
                        {
                            /*
                             * If we reach the end of the expected lines, we
                             * store the rest in the extra config string.
                             */
                            break;
                        }
                        /*
                         * Only increment the line count if we actually
                         * parsed it.
                         */
                        line++;
                    }
                    /*
                     * If this flow is a muxed group, add the muxing flag.
                     */
                    if (muxed_groups.count(info.group) > 0)
                    {
                        info.static_flags |= telem_flag_t::telem_mux_strings;
                    }
                    /*
                     * Store all remaining lines in the config info string.
                     */
                    while (line < file_lines.size())
                    {
                        const str_v &curr_line = file_lines[line];
                        FswAbortIfNot(curr_line.size() > 0, false);
                        /*
                         * "flow" lines start new flow definitions.
                         */
                        if (curr_line[0] == "flow")
                            break;
                        /*
                         * Store config line.
                         */
                        for (uint i = 0; i < curr_line.size(); i++)
                        {
                            if (i > 0)
                                info.config_info += " ";
                            info.config_info += curr_line[i];
                        }
                        info.config_info += "\n";
                        line++;
                    }
                    /*
                     * Store complete telemetry flow info object.
                     */
                    flow_info.push_back(info);
                }
                return true;
            }

        protected:
            /**
             * Parse the external_record flag line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            virtual bool parse_er_line(const str_v &line,
                                       TelemetryFlowInfo &info)
            {
                /*
                 * Control code doesn't care, do nothing.
                 */
                return true;
            }

        private:
            /**
             * Parse the header line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_header_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if (!(line.size() == 3) || line[0] != "flow")
                {
                    FswPrefix();
                    dbstring(": Expecting \"flow <id> <group>\".\n");
                    return false;
                }
                /*
                 * Get ID.
                 */
                uint temp_id;
                if (!string_to_uint(line[1], temp_id))
                {
                    FswPrefix();
                    dbnprintf(100, ": \"%s\" is not a valid telemetry type.\n",
                              line[1].c_str());
                    return false;
                }
                info.id = temp_id;
                /*
                 * Get group.
                 */
                uint temp_group;
                if (!telem_group_t_sym.raw_get("telem_group_" + line[2],
                                               temp_group))
                {
                    FswPrefix();
                    dbnprintf(100, ": \"%s\" is not a known telemetry name.\n",
                              line[2].c_str());
                    return false;
                }
                FswAbortOutsideRangeUint(temp_group, 0, last_telem_group_t,
                                         false);
                info.group = (telem_group_t)temp_group;
                return true;
            }
            /**
             * Parse the type line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_type_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if (!(line.size() == 2) || line[0] != "type")
                {
                    FswPrefix();
                    dbstring(": Expecting \"type <type>\".\n");
                    return false;
                }
                /*
                 * Get type.
                 */
                uint temp_type;
                if (!telem_type_t_sym.raw_get("telem_" + line[1], temp_type))
                {
                    FswPrefix();
                    dbnprintf(100, ": \"%s\" is not a valid telemetry type.\n",
                              line[1].c_str());
                    return false;
                }
                FswAbortOutsideRangeUint(temp_type, 0, last_telem_type_t,
                                         false);
                info.type = (telem_type_t)temp_type;
                return true;
            }
            /**
             * Parse the destination line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_destination_line(const str_v &line,
                                        TelemetryFlowInfo &info)
            {
                if ((line.size() < 2) || line[0] != "destination")
                {
                    FswPrefix();
                    dbstring(": Expecting \"destination <host1>"
                             " [host2] ... [hostN]\".\n");
                    return false;
                }
                /*
                 * Hostname set used to detect duplicate destination entries.
                 */
                std::set<std::string> hostnames;
                /*
                 * Get destination hosts.
                 */
                for (size_t i = 1; i < line.size(); i++)
                {
                    const std::string &host = line[i];
                    FswAbortIfNot(hostnames.find(host) == hostnames.end(),
                                  false);
                    info.hosts[0].push_back(host);
                    hostnames.insert(host);
                }
                return true;
            }
            /**
             * Parse the bandwidth line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_bandwidth_line(const str_v &line,
                                      TelemetryFlowInfo &info)
            {
                if ((line.size() != 3) || line[0] != "bandwidth")
                {
                    FswPrefix();
                    dbstring(": Expecting \"bandwidth <burst> <recharge>\".\n");
                    return false;
                }
                /*
                 * Get bandwidth burst.
                 */
                FswAbortIfNot(string_to_size_t(line[1], info.bandwidth_burst),
                              false);
                FswAbortIfEqInt(info.bandwidth_burst, 0, false);
                /*
                 * Get bandwidth recharge.
                 */
                FswAbortIfNot(string_to_uint(line[2], info.bandwidth_recharge),
                              false);
                FswAbortIfEqInt(info.bandwidth_recharge, 0, false);
                return true;
            }
            /**
             * Parse the idle_timeout line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_timeout_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) || line[0] != "idle_timeout")
                {
                    FswPrefix();
                    dbstring(": Expecting \"idle_timeout <timeout>\".\n");
                    return false;
                }
                /*
                 * Get idle timeout.
                 */
                double idle_timeout_sec;
                FswAbortIfNot(string_to_double(line[1], idle_timeout_sec),
                              false);
                info.idle_timeout = (nano_t)(idle_timeout_sec * dbillion);
                return true;
            }
            /**
             * Parse the store_and_forward flag line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_sf_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) || line[0] != "store_and_forward")
                {
                    FswPrefix();
                    dbstring(": Expecting 'store_and_forward external'.\n");
                    return false;
                }
                if (line[1] == "external")
                {
                    /*
                     * Force set the store and forward flag.
                     *
                     * This is used when flows are stored and forwarded by
                     * systems external to the TelemetryRelayRuntime (such as
                     * the Telemetry RIO on F9 or the StoreAndForwardManager
                     * on Dragon 2).
                     */
                    info.static_flags |= telem_store_and_forward;
                }
                else
                {
                    FswPrefix();
                    dbnprintf(100,
                              ": store_and_forward flag must be 'external', "
                              "not '%s'\n",
                              line[1].c_str());
                    return false;
                }
                return true;
            }
            /**
             * Parse the buffer size line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_buffer_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) || line[0] != "buffer")
                {
                    FswPrefix();
                    dbstring(": Expecting \"buffer <size>\".\n");
                    return false;
                }
                /*
                 * Get buffer size.
                 */
                if (FswIfNot(string_to_size_t(line[1], info.buf_size)))
                {
                    FswPrefix();
                    dbnprintf(100, ": \"%s\" is not a valid buffer size.\n",
                              line[1].c_str());
                    return false;
                }
                return true;
            }
            /**
             * Parse the name line of a config file flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_name_line(const str_v &line, TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) || line[0] != "name")
                {
                    FswPrefix();
                    dbstring(": Expecting \"name <name>\".\n");
                    return false;
                }
                /*
                 * Get the name.
                 */
                info.name = line[1];
                return true;
            }
            /**
             * Parse the disabled destination line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_disabled_destination_line(const str_v &line,
                                                 TelemetryFlowInfo &info)
            {
                if ((line.size() <= 1) || line[0] != "disabled_destination")
                {
                    FswPrefix();
                    dbstring(": Expecting \"disabled_destination <host1>"
                             " [host2] ... [hostN]\".\n");
                    return false;
                }
                /*
                 * Hostname set used to detect duplicate disabled destination
                 * entries.
                 */
                std::set<std::string> hostnames;
                /*
                 * Alt destination is mutually exclusive with disabled
                 * destination because they both allow for ways to modify the
                 * destination of a flow and is not obvious what should happen
                 * if both are selected.
                 */
                for (size_t alt_dest = 1;
                     alt_dest <= TelemetryFlowInfo::num_alt_destinations;
                     alt_dest++)
                {
                    FswMsgAbortIfNot(
                        info.hosts[alt_dest].empty(), false, 200,
                        "Cannot specify disabled_destination when "
                        "alt_destination_%zu was already specified.",
                        alt_dest);
                }
                /*
                 * Get disabled destination hosts.
                 */
                for (size_t i = 1; i < line.size(); i++)
                {
                    const std::string &host = line[i];
                    FswMsgAbortIfNot(
                        hostnames.find(host) == hostnames.end(), false, 200,
                        "Duplicate disabled_destination \"%s\" in flow %i",
                        host.c_str(), info.id);
                    info.disabled_hosts.push_back(host);
                    hostnames.insert(host);
                }
                return true;
            }
            /**
             * Parse the alt destination lines of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_alt_destination_lines(const str_v &line,
                                             TelemetryFlowInfo &info)
            {
                FswAbortIf(line.empty(), false);
                const size_t prefix_len = strlen("alt_destination_");
                /*
                 * 1-based index, because 0 means default.
                 */
                size_t alt_dest = 1;
                if ((line.size() <= 1) ||
                    line[0].compare(0, prefix_len, "alt_destination_") != 0 ||
                    !string_to_size_t(line[0].substr(prefix_len), alt_dest) ||
                    alt_dest == 0 ||
                    alt_dest > TelemetryFlowInfo::num_alt_destinations)
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": Expecting \"alt_destination_N <host1> [host2] "
                              "... [hostM]\" (N can be 1 to %zu).\n",
                              TelemetryFlowInfo::num_alt_destinations);
                    return false;
                }
                /*
                 * Alt destination is mutually exclusive with disabled
                 * destination because they both allow for ways to modify the
                 * destination of a flow and is not obvious what should happen
                 * if both are selected.
                 */
                FswMsgAbortIfNot(info.disabled_hosts.empty(), false, 200,
                                 "Cannot specify alt_destination_%zu when "
                                 "disabled_destination was already specified.",
                                 alt_dest);
                /*
                 * Hostname set used to detect duplicate destination
                 * entries.
                 */
                std::set<std::string> hostnames;
                /*
                 * Get destination hosts.
                 */
                for (size_t i = 1; i < line.size(); i++)
                {
                    const std::string &host = line[i];
                    FswMsgAbortIfNot(
                        hostnames.find(host) == hostnames.end(), false, 200,
                        "Duplicate alt_destination_%zu \"%s\" in flow %i",
                        alt_dest, host.c_str(), info.id);
                    info.hosts[alt_dest].push_back(host);
                    hostnames.insert(host);
                }
                return true;
            }
            /**
             * Parse the override source role & instance line of a config file
             * flow description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_override_source_role_inst(const str_v &line,
                                                 TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) ||
                    line[0] != "override_source_role_inst")
                {
                    FswPrefix();
                    dbstring(": Expecting \"override_source_role_inst "
                             "<override>\".\n");
                    return false;
                }
                info.override_source_role_inst = line[1];
                return true;
            }
            /**
             * Parse the precomputed annotation file line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_precomputed_annotation_file(const str_v &line,
                                                   TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) ||
                    line[0] != "precomputed_annotation_file")
                {
                    FswPrefix();
                    dbstring(": Expecting \"precomputed_annotation_file "
                             "<precomputed_annotation_file_name>\".\n");
                    return false;
                }
                info.precomputed_annotation_file_name = line[1];
                return true;
            }
            /**
             * Parse the timestamp_channel line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_timestamp_channel(const str_v &line,
                                         TelemetryFlowInfo &info)
            {
                if ((line.size() != 2) || line[0] != "timestamp_channel")
                {
                    FswPrefix();
                    dbstring(": Expecting \"timestamp_channel "
                             "<slate_channel_name>\".\n");
                    return false;
                }
                info.timestamp_channel = line[1];
                return true;
            }
        };
        /**
         * Performs a complete parse of telemetry using information only
         * accessible to runtime logic.
         */
        class RuntimeFlowInfoParser : public ControlFlowInfoParser
        {
        public:
            /**
             * Constructor.
             *
             * @param ident The node identity.
             */
            RuntimeFlowInfoParser(const NodeIdentity &_ident)
                : ControlFlowInfoParser(), ident(_ident)
            {}

        protected:
            /**
             * Parse the external_record flag line of a config file flow
             * description.
             *
             * @param line Line to parse.
             * @param[out] info Will fill this object with information found.
             *
             * @return True on success.
             */
            bool parse_er_line(const str_v &line,
                               TelemetryFlowInfo &info) override
            {
                if ((line.size() != 2) || line[0] != "external_record")
                {
                    FswPrefix();
                    dbstring(": Expecting \"external_record "
                             "<all/only_a/only_b/only_c/none>\".\n");
                    return false;
                }
                /*
                 * Get flag state.
                 */
                if (line[1] == "all")
                {
                    info.static_flags |= telem_external_record;
                }
                else if (line[1] == "none")
                {
                    info.static_flags &= ~telem_external_record;
                }
                else if (line[1] == "only_a" || line[1] == "only_b" ||
                         line[1] == "only_c")
                {
                    if ((line[1] == "only_a" && ident.string == "a") ||
                        (line[1] == "only_b" && ident.string == "b") ||
                        (line[1] == "only_c" && ident.string == "c"))
                    {
                        info.static_flags |= telem_external_record;
                    }
                    else
                    {
                        info.static_flags &= ~telem_external_record;
                    }
                }
                else
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": external_record flag must be "
                              "all/only_a/only_b/only_c/none not \"%s\"\n",
                              line[1].c_str());
                    return false;
                }
                return true;
            }

        private:
            /**
             * Identity of this node.
             */
            const NodeIdentity ident;
        };
    } // namespace
    /**
     * Constructor.
     *
     * @param _dest_validity The list of valid alt destinations.
     */
    TelemetryAltDestinationValidator::TelemetryAltDestinationValidator(
        const dest_validity_v &_dest_validity)
        : dest_validity(_dest_validity)
    {}
    /**
     * Called when a flow's "destination" slate channel is about to be set.
     *
     * @param new_val The new destination value.
     * @param[in,out] val Reference to the value in Slate.
     *
     * @return True if the new value was accepted.
     */
    bool TelemetryAltDestinationValidator::validate(const UINT8 &new_value,
                                                    UINT8 &value) const
    {
        FswAbortIfNotOpUint(new_value, <, dest_validity.size(), false);
        FswAbortIfNot(dest_validity[new_value], false);
        value = new_value;
        return true;
    }
    /**
     * Default name of the telemetry config file.
     */
    const std::string TelemetryRelay::default_config_key = "telemetry";
    /**
     * Constructor.
     */
    TelemetryRelayControl::TelemetryRelayControl() : dest_validators() {}
    /**
     * Initialize any control slate elements from a config file.
     *
     * @param name Name of the relay.
     * @param control_builder Root of the control slate.
     * @param configs Config file finder.
     * @param config_key Name of the telemetry config file.
     * @param use_sync_shard If true, create slate elements in the sync shard.
     *                       Otherwise, use the nonsync shard.
     *
     * @return True on success.
     */
    bool TelemetryRelayControl::init(const std::string &name,
                                     SlateBuilder control_builder,
                                     const Configs &configs,
                                     const std::string &config_key,
                                     const bool use_sync_shard)
    {
        /*
         * Parse telemetry configs to get a list of all flows.
         */
        std::string filename;
        FswAbortIfNot(configs.config_file(config_key, filename), false);
        str_v_v file_lines;
        FswAbortIfNot(read_meta_str_v_v(configs, filename, file_lines), false);
        /*
         * Create all necessary control elements for each flow.
         */
        FswAbortIfNot(init(name, control_builder, file_lines, use_sync_shard),
                      false);
        return true;
    }
    /**
     * Initialize any control slate elements from a config string.
     *
     * @param name Name of the relay.
     * @param control_builder Root of the control slate.
     * @param file_lines Contents of the config file to use.
     * @param use_sync_shard If true, create slate elements in the sync shard.
     *                       Otherwise, use the nonsync shard.
     *
     * @return True on success.
     */
    bool TelemetryRelayControl::init(const std::string &name,
                                     SlateBuilder control_builder,
                                     const str_v_v &file_lines,
                                     const bool use_sync_shard)
    {
        /*
         * TelemetryRelayControl does not use the static_flags field in
         * flow_info, so we can ignore muxed groups.
         */
        const std::set<telem_group_t> muxed_groups = {};
        std::vector<TelemetryFlowInfo> flow_info;
        FswAbortIfNot(ControlFlowInfoParser().parse_file(
                          file_lines, muxed_groups, flow_info),
                      false);
        /*
         * Create all necessary control elements for each flow.
         */
        FswAbortIfNot(
            init_internal(name, control_builder, flow_info, use_sync_shard),
            false);
        return true;
    }
    /**
     * TODO: Delete.
     */
    void TelemetryRelayControl::dispatch(const nano_t) RUNTIME {}
    /**
     * Initialize from a list of parsed TelemetryFlowInfo objects.
     *
     * @param name Name of the relay.
     * @param control_builder Root of the control slate.
     * @param flow_info List of TelemetryFlowInfo objects.
     * @param use_sync_shard If true, create slate elements in the sync shard.
     *                       Otherwise, use the nonsync shard.
     *
     * @return True on success.
     */
    bool TelemetryRelayControl::init_internal(
        const std::string &name, SlateBuilder control_builder,
        const std::vector<TelemetryFlowInfo> &flow_info,
        const bool use_sync_shard)
    {
        const slate_shard_t slate_shard =
            use_sync_shard ? shard_sync : shard_nonsync;
        SlateBuilder internal_builder = control_builder.sub_slate(name);
        /*
         * A mapping from flow name to id.  Used for detecting duplicate names.
         */
        std::map<std::string, int> names;
        for (const TelemetryFlowInfo &flow : flow_info)
        {
            /*
             * Check for duplicate names.
             */
            if (!flow.name.empty())
            {
                int duplicate_id = 0;
                FswMsgAbortIf(map_find(names, flow.name, duplicate_id), false,
                              200,
                              "Duplicate flow name \"%s\" on "
                              "flow %i and %i\n",
                              flow.name.c_str(), duplicate_id, flow.id);
            }
            /*
             * Insert our name into the map.
             */
            names[flow.name] = flow.id;
            /*
             * Create the control element if this is a gated flow. Start with
             * it disabled.
             */
            if (!flow.disabled_hosts.empty())
            {
                SlateBuilder sub_builder =
                    internal_builder.sub_slate(flow.name);
                ReadToken<bool> enabled_tok;
                FswAbortIfNot(sub_builder.create("enabled", false, slate_shard,
                                                 slate_read_write, enabled_tok),
                              false);
            }
            /*
             * Create the control element if this is flow with alt destinations.
             */
            bool has_alt_dest = false;
            std::array<bool, TelemetryFlowInfo::num_alt_destinations + 1>
                alt_dest_validity = {};
            alt_dest_validity[0] = true;
            for (size_t alt_dest = 1;
                 alt_dest <= TelemetryFlowInfo::num_alt_destinations;
                 alt_dest++)
            {
                if (!flow.hosts[alt_dest].empty())
                {
                    has_alt_dest = true;
                    alt_dest_validity[alt_dest] = true;
                }
            }
            if (has_alt_dest)
            {
                /*
                 * Create a slate validator for this so we can't accidentally
                 * set destination to the wrong value.
                 */
                Handle<TelemetryAltDestinationValidator> validator(
                    new TelemetryAltDestinationValidator(alt_dest_validity));
                FswAbortIfNot(validator, false);
                dest_validators[flow.id] = validator;
                /*
                 * Create the slate channel for specifying destination.
                 */
                SlateBuilder sub_builder =
                    internal_builder.sub_slate(flow.name);
                ReadToken<UINT8> alt_destination_tok;
                FswAbortIfNot(
                    sub_builder.create(
                        "destination", 0U, slate_shard, slate_read_write,
                        member_validator(
                            *validator,
                            &TelemetryAltDestinationValidator::validate),
                        alt_destination_tok),
                    false);
            }
        }
        /*
         * Create a single Slate entry which is always true. Used as the
         * "is enabled" control element for non-gated flows.
         */
        ReadToken<bool> default_enabled_tok;
        FswAbortIfNot(internal_builder.create("always_enabled", true,
                                              shard_static, slate_read_only,
                                              default_enabled_tok),
                      false);
        /*
         * Create a single Slate entry which is always 0. Used as the
         * "destination" control element for flows with no alt dests
         */
        ReadToken<UINT8> default_alt_destination_tok;
        FswAbortIfNot(internal_builder.create("default_destination", 0U,
                                              shard_static, slate_read_only,
                                              default_alt_destination_tok),
                      false);
        return true;
    }
    /**
     * Construct a TelemetryRelayRuntime.
     *
     * @param _clock The clock used to query time.
     * @param name Control task name.
     */
    TelemetryRelayRuntime::TelemetryRelayRuntime(const Clock &_clock,
                                                 const std::string &_name)
        : name(_name), is_initialized(false), is_finalized(false),
          clock(_clock), slate(), local_ident(), ids(),
          slate_group_flow_indices(), claimed_ids(), id_to_config_info(),
          producers(), flows(), service_redirects(),
          master_override_service("", 0, udp_proto),
          buffer_size_by_destination(), flows_and_alt_dest_by_destination(),
          flows_by_disabled_destination()
    {}
    /**
     * Virtual destructor.
     */
    TelemetryRelayRuntime::~TelemetryRelayRuntime() {}
    /**
     * Initialize this TelemetryRelayRuntime from a config file.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param control_builder Bind to control elements in this slate.
     * @param local_builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param config_key Key of the configuration file to read from.
     * @param _muxed_groups A set of groups that should have the muxing flag
     *                      added.
     * @param should_aggregate_destination_connections If true, this relay will
     *          create one connection per destination, with a buffer sized to
     *          accommodate one of each packet expected to go to that
     *          destination. Otherwise, creates a connection per flow per
     *          destination, with a buffer sized to one packet of that flow.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::init(
        EventLoop &eloop, EventList &elist, SlateBuilder control_builder,
        SlateBuilder local_builder, const Configs &configs,
        const NodeIdentity &ident, const std::string &config_key,
        std::set<telem_group_t> _muxed_groups,
        const bool should_aggregate_destination_connections)
    {
        FswAbortIf(is_initialized, false);
        /*
         * Open file.
         */
        std::string filename;
        FswAbortIfNot(configs.config_file(config_key, filename), false);
        /*
         * Parse lines.
         */
        str_v_v file_lines;
        FswAbortIfNot(read_meta_str_v_v(configs, filename, file_lines), false);
        if (FswIfNot(init(eloop, elist, control_builder, local_builder, configs,
                          ident, file_lines, std::move(_muxed_groups),
                          should_aggregate_destination_connections)))
        {
            FswPrefix();
            dbnprintf(100, ": Failed to parse config file \"%s\".\n",
                      config_key.c_str());
            return false;
        }
        return true;
    }
    /**
     * Initialize this TelemetryRelayRuntime from a line-parsed config file.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param control_builder Bind to control elements in this slate.
     * @param local_builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param file_lines Parsed out lines of the config file.
     * @param _muxed_groups A set of groups that should have the muxing flag
     *                      added.
     * @param should_aggregate_destination_connections If true, this relay will
     *          create one connection per destination, with a buffer sized to
     *          accommodate one of each packet expected to go to that
     *          destination. Otherwise, creates a connection per flow per
     *          destination, with a buffer sized to one packet of that flow.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::init(
        EventLoop &eloop, EventList &elist, SlateBuilder control_builder,
        SlateBuilder local_builder, const Configs &configs,
        const NodeIdentity &ident, const str_v_v &file_lines,
        std::set<telem_group_t> _muxed_groups,
        const bool should_aggregate_destination_connections)
    {
        FswAbortIf(is_initialized, false);
        muxed_groups = std::move(_muxed_groups);
        std::vector<TelemetryFlowInfo> flow_info;
        /*
         * Parse config info.
         */
        FswAbortIfNot(parse_file(file_lines, ident, muxed_groups, flow_info),
                      false);
        FswAbortIfNot(init_internal(eloop, elist, control_builder,
                                    local_builder, configs, ident, flow_info,
                                    should_aggregate_destination_connections),
                      false);
        FswAbortIfNot(local_ident.init(ident.role, ident.inst, ident.string),
                      false);
        return true;
    }
    /**
     * Parse a configuration file.
     *
     * @param file_lines Lines of the config file.
     * @param ident The node identity.
     * @param muxed_groups A set of groups that should have the muxing flag
     *                     added.
     * @param[out] flow_info Returns list of TelemetryFlowInfo objects.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::parse_file(
        const str_v_v &file_lines, const NodeIdentity &ident,
        const std::set<telem_group_t> &muxed_groups,
        std::vector<TelemetryFlowInfo> &flow_info)
    {
        FswAbortIfNot(RuntimeFlowInfoParser(ident).parse_file(
                          file_lines, muxed_groups, flow_info),
                      false);
        return true;
    }
    /**
     * Finalize configuration.
     *
     * @param smoketest_config The smoketest config to reference during
     * finalization.
     *
     * @return True on success.
     */
    bool
    TelemetryRelayRuntime::finalize(const SmoketestConfig &smoketest_config)
    {
        FswAbortIfNot(is_initialized, false);
        FswAbortIf(is_finalized, false);
        /*
         * Make sure all flows were claimed.
         */
        for (group_id_v_m::const_iterator iter = ids.begin(); iter != ids.end();
             ++iter)
        {
            const std::vector<telem_id_t> &group_ids = iter->second;
            for (size_t i = 0; i < group_ids.size(); ++i)
            {
                const telem_id_t id = group_ids[i];
                if (claimed_ids.find(id) == claimed_ids.end())
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": Cannot start telemetry relay, flow %d "
                              "for %s was not claimed.\n",
                              id, telem_group_t_sym.get(iter->first).c_str());
                    return false;
                }
            }
        }
        /*
         * If we are in smoketest, claim the flow ids to assert that they are
         * not used elsewhere on this node.
         */
        for (const auto flow_id : claimed_ids)
        {
            FswAbortIfNot(
                smoketest_config.claim_flow_id(local_ident.node_name, flow_id),
                false);
        }
        is_finalized = true;
        return true;
    }
    /**
     * Get the set of telemetry flow IDs for the given telemetry group.
     *
     * @param group Telemetry group to get IDs for.
     * @param[out] _ids Returns telemetry flow IDs.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::get_ids(telem_group_t group,
                                        std::vector<telem_id_t> &_ids) const
    {
        FswAbortIfNot(is_initialized, false);
        FswAbortIf(is_finalized, false);
        const group_id_v_m::const_iterator iter = ids.find(group);
        if (iter != ids.end())
        {
            _ids = iter->second;
        }
        else
        {
            _ids.clear();
        }
        return true;
    }
    /**
     * Get the configuration info for a given flow.
     *
     * @param id ID of flow.
     * @param[out] flow_info Receives the configuration info.
     *
     * @return True on success.
     */
    bool
    TelemetryRelayRuntime::get_flow_info(const telem_id_t id,
                                         TelemetryFlowInfo &flow_info) const
    {
        FswAbortIfNot(is_initialized, false);
        FswAbortIf(is_finalized, false);
        FswAbortIfNot(map_find(id_to_config_info, id, flow_info), false);
        return true;
    }
    /**
     * Claim a telemetry flow through a TelemetryWriter.
     *
     * @param id ID of flow to claim.
     * @param[out] flow_phase_counter Additional phase offset for round robin.
     * @param[out] flow_info Configuration info for this flow.
     * @param[out] writer Will connect this writer to the requested flow.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::claim(telem_id_t id, size_t &flow_phase_counter,
                                      TelemetryFlowInfo &flow_info,
                                      TelemetryWriter &writer)
    {
        FswAbortIfNot(is_initialized, false);
        FswAbortIf(is_finalized, false);
        FswAbortIf(writer.is_good(), false);
        /*
         * Ensure the ID has not already been claimed.
         */
        if (claimed_ids.find(id) != claimed_ids.end())
        {
            FswPrefix();
            dbnprintf(
                100, ": Telemetry flow \"%d\" has already been claimed.\n", id);
            return false;
        }
        claimed_ids.insert(id);
        Handle<TelemetryRelayFlow> flow;
        FswAbortIfNot(get_flow(id, flow), false);
        /*
         * Connect writer.
         */
        Handle<TelemetryConsumer> consumer_handle = flow;
        FswAbortIfNot(writer.assign_consumer(consumer_handle), false);
        flow_info = id_to_config_info[id];
        flow_phase_counter = 0;
        if (flow_info.group == telem_group_t::telem_group_slate)
        {
            /*
             * If flow is in the slate group, add a monotonic phase offset.
             */
            flow_phase_counter = slate_group_flow_indices[id];
        }
        return true;
    }
    /**
     * Add a telemetry producer to be dispatched at the same time as this
     * TelemetryRelayRuntime.
     *
     * It is advantageous to connect telemetry producer tasks directly to
     * the TelemetryRelayRuntime instead of an EventList because you can be
     * guaranteed that the task will run after all tasks before the relay
     * in the EventList, but BEFORE the relay itself is run (if producers
     * run after the relay, message transmission may be delayed until the
     * next control cycle due to buffering).
     *
     * @param producer Telemetry producer EventSource to add.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::add_producer(Handle<TelemetryTask> producer)
    {
        FswAbortIf(is_finalized, false);
        FswAbortIfNot(producer, false);
        producers.push_back(producer);
        return true;
    }
    /**
     * Add a master override address. This will redirect all telemetry
     * to this address.
     *
     * @note This method must be called *before* init() to supersede any
     * connections.
     *
     * @param host_port Override address for all connections.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::override_addresses(const std::string &host_port)
    {
        FswAbortIf(is_initialized, false);
        FswAbortIf(is_finalized, false);
        FswAbortIfNeqString(master_override_service.host_name, "", false);
        std::string host = "unknown";
        in_port_t port = 0;
        FswAbortIfNot(
            string_to_address(host_port, telemetry_vehicle_port, host, port),
            false);
        master_override_service = Service(host, port, udp_proto);
        return true;
    }
    /**
     * Redirect data bound for the named service to the provided connection
     * (as opposed to being sent out over UDP directly). This can be used, for
     * example, to redirect through a sockpair instead of a network socket, or
     * for local post-processing.
     *
     * @note This method must be called *before* init() to supersede any
     * connections.
     *
     * @param service_name Name of service to mock out.
     * @param connection Connection object to use.
     *
     * @return True on success.
     */
    bool
    TelemetryRelayRuntime::redirect_service(const std::string &service_name,
                                            Handle<BwpWriter> connection)
    {
        FswAbortIf(is_initialized, false);
        FswAbortIf(is_finalized, false);
        FswAbortIfNot(connection, false);
        FswAbortIfNot(service_redirects.find(service_name) ==
                          service_redirects.end(),
                      false);
        service_redirects[service_name] = connection;
        return true;
    }
    /**
     * Dispatch this TelemetryRelayRuntime.
     *
     * @param control_time Current control time.
     *
     * @return Next wakeup time.
     */
    nano_t TelemetryRelayRuntime::dispatch(nano_t control_time)
    {
        FswAbortIfNot(is_initialized, nano_t_max);
        FswAbortIfNot(is_finalized, nano_t_max);
        nano_t next_wakeup = nano_t_max;
        /*
         * Dispatch the producers. Each producer pushes data into its consumer.
         * Dropping timestamp precision from nanoseconds to microseconds
         * to remove the lower bits -- which are more entropy than signal.
         */
        const uint64_t NANO_TO_MICRO = 1000;
        const nano_t telemetry_time =
            (clock.get_telemetry_timestamp() / NANO_TO_MICRO) * NANO_TO_MICRO;
        for (Handle<TelemetryTask> &producer : producers)
        {
            next_wakeup = std::min(
                producer->dispatch(control_time, telemetry_time), next_wakeup);
        }
        /*
         * Dispatch the consumers. The consumer constructs a fully-formed
         * telemetry message. Each consumer is connected to a channel. When the
         * event loop considers the fd associated with the channel for
         * 'writing', the telemetry message is written.
         */
        for (std::pair<const telem_id_t, Handle<TelemetryRelayFlow>> &flow :
             flows)
        {
            next_wakeup =
                std::min(flow.second->dispatch(control_time), next_wakeup);
        }
        return next_wakeup;
    }
    /**
     * Returns the name of the TelemetryRelay instance.
     *
     * @return Name of this relay.
     */
    const std::string &TelemetryRelayRuntime::get_name() { return name; }
    /**
     * @return A set of telemetry groups that will have the muxing flag added to
     *         their flows.
     */
    const std::set<telem_group_t> &
    TelemetryRelayRuntime::get_muxed_groups() const
    {
        return muxed_groups;
    }
    /**
     * Initialize this TelemetryRelayRuntime from a list of TelemetryFlowInfo
     * objects.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param control_builder Bind to control elements in this slate.
     * @param local_builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param flow_info List of TelemetryFlowInfo objects.
     * @param should_aggregate_destination_connections If true, this relay will
     *          create one connection per destination, with a buffer sized to
     *          accommodate one of each packet expected to go to that
     *          destination. Otherwise, creates a connection per flow per
     *          destination, with a buffer sized to one packet of that flow.
     *
     *          Note: This has the effect of sorting the service names on a
     *          given line of config. Consult the note on the bottom of
     * dat/TelemetryRelay_uto/init_control_and_runtime_config for more
     *          information.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::init_internal(
        EventLoop &eloop, EventList &elist, SlateBuilder control_builder,
        SlateBuilder local_builder, const Configs &configs,
        const NodeIdentity &ident,
        const std::vector<TelemetryFlowInfo> &flow_info,
        const bool should_aggregate_destination_connections)
    {
        FswAbortIf(is_initialized, false);
        /*
         * Create state.
         */
        SlateBuilder internal_control = control_builder.sub_slate(name);
        /*
         * Create flow objects.
         */
        SlateBuilder internal_local = local_builder.sub_slate(name);
        FswAbortIfNot(create_flows(eloop, elist, internal_control,
                                   internal_local, ident, flow_info,
                                   should_aggregate_destination_connections),
                      false);
        slate = internal_local.slate(slate_no_validation);
        is_initialized = true;
        return true;
    }
    /**
     * Get a connection by host/port.
     *
     * @param eloop Reference to the EventLoop (needed to create new
     * connections).
     * @param elist Add tasks to this EventList.
     * @param service_name Service name of connection.
     * @param buffer_size_request If creating a new connection, use this
     * buffer size in bytes.
     * @param[out] connection Returns connection object.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::get_connection(EventLoop &eloop,
                                               EventList &elist,
                                               const std::string &service_name,
                                               const size_t buffer_size_request,
                                               Handle<BwpWriter> &connection)
    {
        FswAbortIf(connection, false);
        Service service("", 0, udp_proto);
        /*
         * If the override connection exists, we use it.
         */
        if (!master_override_service.host_name.empty())
        {
            service = master_override_service;
        }
        else
        {
            const ServiceDirectory &sd = service_directory();
            if (FswIfNot(sd.lookup(service_name, service)))
            {
                FswPrefix();
                dbnprintf(100, ": Service '%s' was not found.\n",
                          service_name.c_str());
                return false;
            }
        }
        /*
         * If this service is being redirected, return the appropriate channel.
         */
        const str_conn_m::iterator iter = service_redirects.find(service_name);
        if (iter != service_redirects.end())
        {
            connection = iter->second;
            return true;
        }
        /*
         * Otherwise create the connection.
         */
        FswAbortIfNot(create_connection(eloop, elist, service,
                                        buffer_size_request, connection),
                      false);
        return true;
    }
    /**
     * Create a new connection to the specified address.
     *
     * @param eloop EventLoop.
     * @param elist Add tasks to this EventList.
     * @param service Service to connect to.
     * @param buffer_size Use this buffer size in bytes.
     * @param[out] connection Returns connection object.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::create_connection(EventLoop &eloop,
                                                  EventList &elist,
                                                  const Service &service,
                                                  const size_t buffer_size,
                                                  Handle<BwpWriter> &connection)
    {
        /*
         * Must be UDP.
         */
        FswAbortIfNot(udp_proto == service.proto, false);
        const size_t num_dgrams = (buffer_size + bwp_datagram_packet_len - 1) /
                                  bwp_datagram_packet_len;
        /*
         * Construct writer.
         */
        Handle<AnyDgramConnection> dgram_conn(new AnyDgramConnection(
            elist, eloop.fds, num_dgrams, bwp_datagram_packet_len));
        FswAbortIfNot(dgram_conn, false);
        Handle<BwpChannelWriter> writer(new BwpChannelWriter);
        FswAbortIfNot(writer, false);
        FswAbortIfNot(dgram_conn->open(service.host_name, service.port), false);
        FswAbortIfNot(writer->assign_channel(dgram_conn), false);
        connection = writer;
        return true;
    }
    /**
     * Create telemetry flows based on information objects.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param control_builder Bind to control elements in this slate.
     * @param local_builder Store state in this slate.
     * @param ident The node identity.
     * @param flow_info List of TelemetryFlowInfo objects.
     * @param should_aggregate_destination_connections If true, this relay will
     *          create one connection per destination, with a buffer sized to
     *          accommodate one of each packet expected to go to that
     *          destination. Otherwise, creates a connection per flow per
     *          destination, with a buffer sized to one packet of that flow.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::create_flows(
        EventLoop &eloop, EventList &elist, SlateBuilder &control_builder,
        SlateBuilder &local_builder, const NodeIdentity &ident,
        const std::vector<TelemetryFlowInfo> &flow_info,
        const bool should_aggregate_destination_connections)
    {
        /*
         * Bind to the default control element for non-gated flows.
         */
        ReadToken<bool> default_enabled_tok;
        FswAbortIfNot(
            control_builder.bind("always_enabled", default_enabled_tok), false);
        /*
         * Bind to the default control element for flows with no alt dest.
         */
        ReadToken<UINT8> default_alt_destination_tok;
        FswAbortIfNot(control_builder.bind("default_destination",
                                           default_alt_destination_tok),
                      false);
        for (const TelemetryFlowInfo &flow : flow_info)
        {
            /*
             * Add ID to group.
             */
            ids[flow.group].push_back(flow.id);
            if (flow.group == telem_group_t::telem_group_slate)
            {
                /*
                 * This flow is at index length(number of slate flows) - 1.
                 */
                slate_group_flow_indices[flow.id] = size(ids[flow.group]) - 1;
            }
            /*
             * Add config info to map.
             */
            if (FswIfNot(id_to_config_info.find(flow.id) ==
                         id_to_config_info.end()))
            {
                FswPrefix();
                dbnprintf(200, ": Flow \"%d\" defined multiple times.\n",
                          flow.id);
                return false;
            }
            id_to_config_info[flow.id] = flow;
            /*
             * Look up the slate element for this gated flow (if set).
             */
            const bool is_gated = !flow.disabled_hosts.empty();
            ReadToken<bool> enabled_tok;
            if (!is_gated)
            {
                enabled_tok = default_enabled_tok;
            }
            else
            {
                /*
                 * Ensure that this flow has the same number of normal, and
                 * disabled, destinations.  This keeps network utilization
                 * consistent regardless of the enabled state (assuming the same
                 * number of unicast/multicast destinations within the two
                 * sets).
                 */
                FswMsgAbortIfNot(
                    flow.hosts[0].size() == flow.disabled_hosts.size(), false,
                    200,
                    "Flow %i has unequal 'destination' and "
                    "'disabled_destination' sizes (%zu and %zu)\n",
                    flow.id, flow.hosts[0].size(), flow.disabled_hosts.size());
                FswMsgAbortIf(flow.name.empty(), false, 200,
                              "Flow %i specifies 'disabled_destination' but "
                              "not 'name'\n",
                              flow.id);
                /*
                 * Bind to the control element for this gated flow.
                 */
                SlateBuilder sub_builder = control_builder.sub_slate(flow.name);
                FswAbortIfNot(sub_builder.bind("enabled", enabled_tok), false);
            }
            /*
             * Look up the slate element for the alt destination for this flow
             * (if set).
             */
            ReadToken<UINT8> alt_destination_tok;
            bool has_alt_dest = false;
            for (size_t alt_dest = 1;
                 alt_dest <= TelemetryFlowInfo::num_alt_destinations;
                 alt_dest++)
            {
                if (!flow.hosts[alt_dest].empty())
                {
                    has_alt_dest = true;
                    /*
                     * Ensure that this flow has the same number of destinations
                     * for normal, and each alt destination configuration.  This
                     * keeps network utilization consistent regardless of the
                     * enabled state (assuming the same number of
                     * unicast/multicast destinations within the two sets).
                     */
                    FswMsgAbortIfNot(
                        flow.hosts[0].size() == flow.hosts[alt_dest].size(),
                        false, 200,
                        "Flow %i has unequal 'destination' and "
                        "'alt_destination_%zu' sizes (%zu and %zu)\n",
                        flow.id, alt_dest, flow.hosts[0].size(),
                        flow.hosts[alt_dest].size());
                    FswMsgAbortIf(flow.name.empty(), false, 200,
                                  "Flow %i specifies 'alt_destination_%zu' but "
                                  "not 'name'\n",
                                  flow.id, alt_dest);
                }
            }
            if (!has_alt_dest)
            {
                alt_destination_tok = default_alt_destination_tok;
            }
            else
            {
                /*
                 * Bind to the control element for this flow with an alt
                 * destination.
                 */
                SlateBuilder sub_builder = control_builder.sub_slate(flow.name);
                FswAbortIfNot(
                    sub_builder.bind("destination", alt_destination_tok),
                    false);
            }
            /*
             * Determine the source address for all packets from this flow.
             */
            std::string full_source_name;
            if (flow.override_source_role_inst.empty())
            {
                full_source_name = ident.node_name;
            }
            else
            {
                full_source_name =
                    flow.override_source_role_inst + ident.string;
            }
            UINT32 node_src;
            FswMsgAbortIfNot(
                node_directory().direct_lookup(full_source_name, node_src),
                false, 100, "Unable to determine source address for node: %s",
                full_source_name.c_str());
            /*
             * Create new flow object.
             */
            Handle<TelemetryRelayFlow> relay_flow(new TelemetryRelayFlow(
                clock, flow.id, flow.group, flow.type, enabled_tok,
                alt_destination_tok, node_src, flow.buf_size,
                flow.static_flags));
            /*
             * Initialize the new flow with a flow specific subslate.
             */
            const std::string subslate_name = "flows." + to_string(flow.id);
            SlateBuilder flow_slate_builder =
                local_builder.sub_slate(subslate_name);
            FswAbortIfNot(relay_flow->init_metrics(flow_slate_builder), false);
            /*
             * We do not allow one of these values to be is_initialized without
             * the other one. This is accomplished by the logical XOR below
             * (bangs convert values to boolean, then compare with a NEQ).
             */
            if (!flow.bandwidth_burst != !flow.bandwidth_recharge)
            {
                FswPrefix();
                dbnprintf(100,
                          ": Telemetry flow %d was given an incomplete"
                          " bandwidth specification\n",
                          flow.id);
                return false;
            }
            /*
             * We default our output buffer to 10 messages, unless we
             * have a bandwidth limit.
             */
            size_t buffer_size = 10 * bwp_datagram_packet_len;
            /*
             * Set bandwidth limit if applicable.
             */
            if (flow.bandwidth_burst)
            {
                FswAbortIfNot(
                    relay_flow->set_bandwidth(flow.bandwidth_burst,
                                              flow.bandwidth_recharge),
                    false);
                const size_t num_dgrams =
                    (flow.bandwidth_burst + bwp_datagram_packet_len - 1) /
                    bwp_datagram_packet_len;
                buffer_size = num_dgrams * bwp_datagram_packet_len;
            }
            FswAbortIfNot(relay_flow->set_idle_timeout(flow.idle_timeout),
                          false);
            /*
             * Create the connections for this flow, if this relay isn't
             * aggregating them, or tally up their buffer size requirements if
             * this relay is aggregating connections.
             */
            for (size_t alt_dest = 0; alt_dest < flow.hosts.size(); alt_dest++)
            {
                for (size_t idx = 0; idx < flow.hosts[alt_dest].size(); idx++)
                {
                    if (should_aggregate_destination_connections)
                    {
                        /*
                         * Tally up buffer size requests by
                         * destination.
                         */
                        buffer_size_by_destination[flow.hosts[alt_dest][idx]] +=
                            buffer_size;
                        /*
                         * Note this flow x index wanted to have a connection to
                         * that destination so we can connect it later.
                         */
                        flows_and_alt_dest_by_destination[flow.hosts[alt_dest]
                                                                    [idx]]
                            .insert(std::pair(flow.id, alt_dest));
                    }
                    else
                    {
                        Handle<BwpWriter> writer;
                        FswAbortIfNot(get_connection(eloop, elist,
                                                     flow.hosts[alt_dest][idx],
                                                     buffer_size, writer),
                                      false);
                        FswAbortIfNot(
                            relay_flow->add_connection(writer, alt_dest),
                            false);
                    }
                }
            }
            /*
             * For gated flows, we also want to add disabled connections
             * for them to send messages to when they are disabled (unless we're
             * aggregating those connections, in which case we still just want
             * to accumulate their buffer need).
             */
            if (is_gated)
            {
                for (size_t idx = 0; idx < flow.disabled_hosts.size(); idx++)
                {
                    if (should_aggregate_destination_connections)
                    {
                        /*
                         * Tally up buffer size requests by
                         * destination.
                         */
                        buffer_size_by_destination[flow.disabled_hosts[idx]] +=
                            buffer_size;
                        /*
                         * Note this flow wanted to have a connection to that
                         * disabled destination so we can connect it later.
                         */
                        flows_by_disabled_destination[flow.disabled_hosts[idx]]
                            .insert(flow.id);
                    }
                    else
                    {
                        Handle<BwpWriter> writer;
                        FswAbortIfNot(get_connection(eloop, elist,
                                                     flow.disabled_hosts[idx],
                                                     buffer_size, writer),
                                      false);
                        FswAbortIfNot(
                            relay_flow->add_disabled_connection(writer), false);
                    }
                }
            }
            /*
             * Add flow to list.
             */
            flows[flow.id] = relay_flow;
        }
        if (should_aggregate_destination_connections)
        {
            /*
             * Iterate over destinations that we tallied up buffer need for
             * previously.
             */
            for (auto const &dest_and_size : buffer_size_by_destination)
            {
                std::string service_name = dest_and_size.first;
                size_t buffer_size = dest_and_size.second;
                /*
                 * Make one connection to that destination.
                 */
                Handle<BwpWriter> writer;
                FswAbortIfNot(get_connection(eloop, elist, service_name,
                                             buffer_size, writer),
                              false);
                if (flows_and_alt_dest_by_destination.find(service_name) !=
                    flows_and_alt_dest_by_destination.end())
                {
                    /*
                     * If that destination was any flow's alt destination...
                     */
                    for (auto flow_id_and_alt_dest :
                         flows_and_alt_dest_by_destination[service_name])
                    {
                        /*
                         * ...iterate over all the flow-dest pairs that we
                         * logged as wanting that destination and give those
                         * flows the previously-made connection.
                         */
                        FswAbortIfNot(
                            flows[flow_id_and_alt_dest.first]->add_connection(
                                writer, flow_id_and_alt_dest.second),
                            false);
                    }
                }
                if (flows_by_disabled_destination.find(service_name) !=
                    flows_by_disabled_destination.end())
                {
                    /*
                     * If that destination was any flow's disabled
                     * destination...
                     */
                    for (auto flow_id :
                         flows_by_disabled_destination[service_name])
                    {
                        /*
                         * ...iterate over all the flows that wanted that
                         * disabled destination and give those flows the
                         * previously-made connection.
                         */
                        FswAbortIfNot(
                            flows[flow_id]->add_disabled_connection(writer),
                            false);
                    }
                }
            }
        }
        return true;
    }
    /**
     * Get a TelemetryRelayFlow by ID.
     *
     * @param id ID of flow to get.
     * @param[out] flow Returns flow object.
     *
     * @return True on success.
     */
    bool TelemetryRelayRuntime::get_flow(telem_id_t id,
                                         Handle<TelemetryRelayFlow> &flow)
    {
        FswAbortIfNot(is_initialized, false);
        FswAbortIf(flow, false);
        const std::map<telem_id_t, Handle<TelemetryRelayFlow>>::iterator iter =
            flows.find(id);
        if (iter == flows.end())
            return false;
        Handle<TelemetryRelayFlow> &temp_flow = iter->second;
        FswAbortIfNot(temp_flow, false);
        flow = temp_flow;
        return true;
    }
    /**
     * Construct a TelemetryRelay.
     *
     * @param _clock The clock used to query time.
     * @param name Control task name.
     */
    TelemetryRelay::TelemetryRelay(const Clock &_clock,
                                   const std::string &_name)
        : TelemetryRelayRuntime(_clock, _name), control()
    {}
    /**
     * Initialize this TelemetryRelayRuntime from a config file.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param config_key Key of the configuration file to read from.
     * @param _muxed_groups A set of groups that should have the muxing flag
     *                      added.
     *
     * @return True on success.
     */
    bool TelemetryRelay::init(EventLoop &eloop, EventList &elist,
                              SlateBuilder builder, const Configs &configs,
                              const NodeIdentity &ident,
                              const std::string &config_key,
                              std::set<telem_group_t> _muxed_groups)
    {
        FswAbortIfNot(TelemetryRelayRuntime::init(
                          eloop, elist, builder, builder, configs, ident,
                          config_key, std::move(_muxed_groups)),
                      false);
        return true;
    }
    /**
     * Initialize this TelemetryRelayRuntime from a line-parsed config file.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param file_lines Parsed out lines of the config file.
     * @param _muxed_groups A set of groups that should have the muxing flag
     *                      added.
     *
     * @return True on success.
     */
    bool TelemetryRelay::init(EventLoop &eloop, EventList &elist,
                              SlateBuilder builder, const Configs &configs,
                              const NodeIdentity &ident,
                              const str_v_v &file_lines,
                              std::set<telem_group_t> _muxed_groups)
    {
        FswAbortIfNot(TelemetryRelayRuntime::init(
                          eloop, elist, builder, builder, configs, ident,
                          file_lines, std::move(_muxed_groups)),
                      false);
        return true;
    }
    /**
     * Dispatch this TelemetryRelayRuntime.
     *
     * @param control_time Current control time.
     *
     * @return Next wakeup time.
     */
    nano_t TelemetryRelay::dispatch(nano_t control_time)
    {
        FswAbortIfNot(control, nano_t_max);
        /*
         * This needs to be done prior to dispatching flows, so data
         * expiration in TelemetryRelayFlow::dispatch has the correct value
         * for the horizon.
         */
        control->dispatch(control_time);
        return TelemetryRelayRuntime::dispatch(control_time);
    }
    /**
     * Initialize from a list of TelemetryFlowInfo objects.
     *
     * @param eloop Reference to the EventLoop.
     * @param elist Add tasks to this EventList.
     * @param control_builder Bind to control elements in this slate.
     * @param local_builder Store state in this slate.
     * @param configs Config file finder.
     * @param ident The node identity.
     * @param flow_info List of TelemetryFlowInfo objects.
     * @param should_aggregate_destination_connections If true, this relay will
     *          create one connection per destination, with a buffer sized to
     *          accommodate one of each packet expected to go to that
     *          destination. Otherwise, creates a connection per flow per
     *          destination, with a buffer sized to one packet of that flow.
     *
     * @return True on success.
     */
    bool TelemetryRelay::init_internal(
        EventLoop &eloop, EventList &elist, SlateBuilder control_builder,
        SlateBuilder local_builder, const Configs &configs,
        const NodeIdentity &ident,
        const std::vector<TelemetryFlowInfo> &flow_info,
        const bool should_aggregate_destination_connections)
    {
        /*
         * Create our own control logic prior to initializing the runtime, but
         * create the control elements in the nonsync shard.
         */
        const bool use_sync_shard = false;
        FswAbortIfNot(control.assume_ownership(new TelemetryRelayControl()),
                      false);
        FswAbortIfNot(control->init_internal(name, control_builder, flow_info,
                                             use_sync_shard),
                      false);
        FswAbortIfNot(TelemetryRelayRuntime::init_internal(
                          eloop, elist, control_builder, local_builder, configs,
                          ident, flow_info,
                          should_aggregate_destination_connections),
                      false);
        return true;
    }
} /* end namespace Drone */