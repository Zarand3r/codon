TelemetryRelay

/**
 * @author Stefan Moluf
 * @date   12/18/09
 */
#ifndef TELEMETRY_RELAY_H
#define TELEMETRY_RELAY_H
#include "src/bullwinkle/all/BwpWriter.h"
#include "src/bullwinkle/all/Configs.h"
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/EventLoop.h"
#include "src/bullwinkle/all/ServiceDirectory.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/SmoketestConfig.h"
#include "src/bullwinkle/all/StringFragmenter.h"
#include "src/bullwinkle/all/TelemetryFlowInfo.h"
#include "src/bullwinkle/all/TelemetryRelayFlow.h"
#include "src/bullwinkle/all/TelemetryWriter.h"
#include "src/bullwinkle/all/identity_util.h"
#include "src/bullwinkle/all/net.h"
#include <map>
#include <set>
#include <utility>
    /*
     * Forward-declare the unit test class so it can be friended.
     */
    class TelemetryRelayUto;
namespace Drone
{
    /**
     * Telemetry producer base class.
     */
    class TelemetryTask
    {
    public:
        /**
         * Destructor.
         */
        virtual ~TelemetryTask() {}
        /**
         * Dispatch call to be overridden by each producer implementation.
         *
         * @param control_time The current control time.
         * @param telem_time The current telemetry time.
         *
         * @return The next wakeup time.
         */
        virtual nano_t dispatch(nano_t control_time, nano_t telem_time) = 0;
    };
    /**
     * Slate validator for destination value for flows with alt destinations.
     * This helps prevent us from setting the wrong destination by mistake as
     * only a few destinations will be populated.
     */
    struct TelemetryAltDestinationValidator : SignalHandler
    {
    public:
        /**
         * List of validity for the different destinations.
         */
        typedef std::array<bool, TelemetryFlowInfo::num_alt_destinations + 1>
            dest_validity_v;
        dest_validity_v dest_validity;
        TelemetryAltDestinationValidator(const dest_validity_v &_dest_validity);
        bool validate(const UINT8 &new_value, UINT8 &value) const;
    };
    /**
     * Manages the synced control logic associated with a TelemetryRelay.
     *
     * The logic in TelemetryRelayRuntime consumes the outputs generated here.
     */
    class TelemetryRelayControl : public SignalHandler
    {
    public:
        TelemetryRelayControl();
        bool init(const std::string &name, SlateBuilder control_builder,
                  const Configs &configs, const std::string &config_key,
                  const bool use_sync_shard = true);
        bool init(const std::string &name, SlateBuilder control_builder,
                  const str_v_v &file_lines, const bool use_sync_shard = true);
        void dispatch(const nano_t control_time) RUNTIME;

    private:
        friend class TelemetryRelay;
        bool init_internal(const std::string &name,
                           SlateBuilder control_builder,
                           const std::vector<TelemetryFlowInfo> &flow_info,
                           const bool use_sync_shard = true);
        /**
         * List of slate validators for flows with alt destinations.
         */
        typedef std::map<telem_id_t, Handle<TelemetryAltDestinationValidator>>
            dest_validator_m;
        dest_validator_m dest_validators;
    };
    /**
     * Manages the unsynced runtime logic associated with a TelemetryRelay.
     *
     * This class must be used in conjunction with an instance of
     * TelemetryRelayControl, which should be initialized and dispatched prior
     * to this class.
     */
    class TelemetryRelayRuntime : public SignalHandler
    {
    public:
        explicit TelemetryRelayRuntime(
            const Clock &_clock, const std::string &_name = "telem_relay");
        virtual ~TelemetryRelayRuntime();
        bool init(EventLoop &eloop, EventList &elist,
                  SlateBuilder control_builder, SlateBuilder local_builder,
                  const Configs &configs, const NodeIdentity &ident,
                  const std::string &config_key = "telemetry",
                  std::set<telem_group_t> _muxed_groups = {},
                  const bool should_aggregate_destination_connections = false);
        bool init(EventLoop &eloop, EventList &elist,
                  SlateBuilder control_builder, SlateBuilder local_builder,
                  const Configs &configs, const NodeIdentity &ident,
                  const str_v_v &file_lines,
                  std::set<telem_group_t> _muxed_groups = {},
                  const bool should_aggregate_destination_connections = false);
        static bool parse_file(const str_v_v &file_lines,
                               const NodeIdentity &ident,
                               const std::set<telem_group_t> &muxed_groups,
                               std::vector<TelemetryFlowInfo> &flow_info);
        bool finalize(const SmoketestConfig &smoketest_config);
        bool get_ids(telem_group_t group, std::vector<telem_id_t> &_ids) const;
        bool get_flow_info(const telem_id_t id,
                           TelemetryFlowInfo &flow_info) const;
        bool claim(telem_id_t id, size_t &flow_phase_counter,
                   TelemetryFlowInfo &flow_info, TelemetryWriter &writer);
        bool add_producer(Handle<TelemetryTask> producer);
        bool override_addresses(const std::string &host_port);
        bool redirect_service(const std::string &service_name,
                              Handle<BwpWriter> connection);
        virtual nano_t dispatch(nano_t control_time);
        const std::string &get_name();
        const std::set<telem_group_t> &get_muxed_groups() const;

    protected:
        virtual bool
        init_internal(EventLoop &eloop, EventList &elist,
                      SlateBuilder control_builder, SlateBuilder local_builder,
                      const Configs &configs, const NodeIdentity &ident,
                      const std::vector<TelemetryFlowInfo> &flow_info,
                      const bool should_aggregate_destination_connections);
        /**
         * Name of this TelemetryRelay instance.
         */
        const std::string name;

    private:
        /*
         * Make friends with the unit test.
         */
        friend class ::TelemetryRelayUto;
        bool init_sf_groups(SlateBuilder builder,
                            const std::vector<TelemetryFlowInfo> &flow_info);
        bool get_connection(EventLoop &eloop, EventList &elist,
                            const std::string &service_name,
                            const size_t buffer_size_request,
                            Handle<BwpWriter> &connection);
        bool create_connection(EventLoop &eloop, EventList &elist,
                               const Service &service, const size_t buffer_size,
                               Handle<BwpWriter> &connection);
        bool create_flows(EventLoop &eloop, EventList &elist,
                          SlateBuilder &control_builder,
                          SlateBuilder &local_builder,
                          const NodeIdentity &ident,
                          const std::vector<TelemetryFlowInfo> &flow_info,
                          const bool should_aggregate_destination_connections);
        bool get_flow(telem_id_t id, Handle<TelemetryRelayFlow> &flow);
        /**
         * True if initialized.
         */
        bool is_initialized;
        /**
         * True if finalized.
         */
        bool is_finalized;
        /**
         * The clock used to query the current time.
         */
        const Clock &clock;
        /**
         * The slate.
         */
        INFRASTRUCTURE(Slate) slate;
        /**
         * A local copy of NodeIdentity.
         */
        NodeIdentity local_ident;
        /**
         * Map of telemetry IDs by group.
         */
        typedef std::map<telem_group_t, std::vector<telem_id_t>> group_id_v_m;
        group_id_v_m ids;
        /**
         * Map of flow index by slate group by telemetry flow ID.
         */
        std::map<telem_id_t, size_t> slate_group_flow_indices;
        /**
         * Set of previously claimed flow IDs.
         */
        typedef std::set<int> int_s;
        int_s claimed_ids;
        /**
         * Map of config info by flow ID.
         */
        typedef std::map<int, TelemetryFlowInfo> id_config_info_m;
        id_config_info_m id_to_config_info;
        /**
         * List of telemetry producers to dispatch.
         */
        std::vector<Handle<TelemetryTask>> producers;
        /**
         * Map of telemetry flow IDs to telemetry consumers.
         */
        std::map<telem_id_t, Handle<TelemetryRelayFlow>> flows;
        /**
         * A map from host name to a BwpWriter Handle that will be used to
         * mock out any connections to that host.
         */
        typedef std::map<std::string, Handle<BwpWriter>> str_conn_m;
        str_conn_m service_redirects;
        /**
         * A service that, if set, will override all connections.
         */
        Service master_override_service;
        /**
         * A set of telemetry groups that will have the muxing flag added to
         * their flows.
         */
        std::set<telem_group_t> muxed_groups;
        /**
         * A map of destinations to size_t, used for accumulating the total
         * buffer size to be used for this destination as the sum of the buffer
         * allocation needed for each flow sending to it.
         */
        std::map<std::string, size_t> buffer_size_by_destination;
        /**
         * A map of destination service names to a set of flow
         * ids/alt-destination pairs, used for assigning single destination
         * connections to many flows.
         */
        std::map<std::string, std::set<std::pair<telem_id_t, size_t>>>
            flows_and_alt_dest_by_destination;
        /**
         * A map of disabled destination service names to a set of flow ids,
         * used for assigning single destination connections to many flows.
         */
        std::map<std::string, std::set<telem_id_t>>
            flows_by_disabled_destination;
    };
    /**
     * The TelemetryRelay is the gatekeeper for all telemetry emitted from a
     * particular program.
     *
     * The core function of the TelemetryRelay, to transmit telemetry frames,
     * is handled by TelemetryRelayFlow objects (one for each flow) which
     * handle data framing, rate limiting, and transmission through a set of
     * BwpWriters. The TelemetryRelay connects these TelemetryRelayFlow
     * objects to TelemetryWriter objects through the claim() interface.
     *
     * This class wraps the functionality of TelemetryRelayControl and
     * TelemetryRelayRuntime into a single package, which is useful for
     * unsynced processes that don't need to distinguish between the two.
     * Synced process should separately create instances of
     * TelemetryRelayControl and TelemetryRelayRuntime.
     */
    class TelemetryRelay : public TelemetryRelayRuntime
    {
    public:
        static const std::string default_config_key;
        explicit TelemetryRelay(const Clock &_clock,
                                const std::string &_name = "telem_relay");
        bool init(EventLoop &eloop, EventList &elist, SlateBuilder builder,
                  const Configs &configs, const NodeIdentity &ident,
                  const std::string &config_key = "telemetry",
                  std::set<telem_group_t> _muxed_groups = {});
        bool init(EventLoop &eloop, EventList &elist, SlateBuilder builder,
                  const Configs &configs, const NodeIdentity &ident,
                  const str_v_v &file_lines,
                  std::set<telem_group_t> _muxed_groups = {});
        nano_t dispatch(nano_t control_time) override;

    private:
        bool init_internal(EventLoop &eloop, EventList &elist,
                           SlateBuilder control_builder,
                           SlateBuilder local_builder, const Configs &configs,
                           const NodeIdentity &ident,
                           const std::vector<TelemetryFlowInfo> &flow_info,
                           const bool should_aggregate_destination_connections =
                               false) override;
        /**
         * The control portion of this object's logic.
         */
        Handle<TelemetryRelayControl> control;
    };
} /* end namespace Drone */
#endif /* TELEMETRY_RELAY_H */
