/**
 * @author Andy Bohn
 * @date   2018-04-22
 */
#ifndef DRONE_FT_RUNTIME_H
#define DRONE_FT_RUNTIME_H
#include "src/bullwinkle/all/AlertBufferManager.h"
#include "src/bullwinkle/all/io/ByteQuotaFramer.h"
#include "src/flight/common/all/CborTools.h"
#include "src/flight/common/all/CommonCommandFilter.h"
#include "src/flight/common/all/FtRuntime.h"
#include "src/flight/sat/all/common/fcpu_board_rev_t.enum.h"
#include "src/flight/sat/all/common/satellite_rev_t.enum.h"
#include "src/flight/sat/all/flight-computer/RealTimeClockInterface.h"
#include "src/flight/sat/all/flight-computer/DroneControl.h"
#include "src/flight/sat/all/flight-computer/TtcRuntime.h"
#include "src/flight/sat/all/flight-computer/WatchdogHeartbeat.h"
#include "src/flight/sat/all/power/DronePowerConverterInterface.h"
#include "src/flight/sat/all/pps/PpsManager.h"
#include "src/flight/sat/all/pps/SwiftPpsInterface.h"
namespace Drone
{
    /**
     * Master runtime system for a Drone Satellite.
     */
    /**
     * A helper structure providing space to stash a const pointer to the
     * init-only data published by the control object.
     */
    struct drone_ft_runtime_init_only_t
    {
        /**
         * A read-only pointer to the init-only data published by the control
         * object.
         */
        std::shared_ptr<const alert_info_init_only_t> alert_info_init_only{};
    };
    class DroneFtRuntime :
        /*
         * drone_ft_runtime_init_only_t must come before FtRuntime, so that
         * drone_ft_runtime_init_only_t::init_only could be referenced during
         * FtRuntime construction.
         */
        private drone_ft_runtime_init_only_t,
        public FtRuntime
    {
    public:
        DroneFtRuntime(EventLoop &_eloop, const Configs &_configs,
                          const Cbor::Entry &_manifest_info,
                          const bool _has_ttc, const bool use_firmware_comm,
                          const CmdLine &_cmd,
                          const bool enable_controlcode_static_analysis);
        ~DroneFtRuntime() override;

    private:
        bool init_runtime_pre_control() override;
        bool init_runtime_pre_slate_build() override;
        bool init_runtime_post_slate_build() override;
        void dispatch_nonsynced_pre_firmware_comm() override;
        void dispatch_nonsynced_pre_telemetry() override;
        bool create_local_telemetry_connections(
            TelemetryRelayRuntime &relay) override;
        bool should_telem_relay_aggregate_dest_connections() const override;
        bool
        create_bootstrapper(Handle<FtBootstrapper> &_bootstrapper) override;
        bool create_command_filters(
            Handle<ExternalCommandTimeFilter> &time_filter,
            Handle<ExternalCommandFilter> &cmd_filter) override;
        bool create_ttc_radio_system();
        bool create_alert_buffer_manager();
        std::string get_telemetry_config_file_name() const override;
        /**
         * The manifest info object read from configs.
         */
        const Cbor::Entry &manifest_info;
        /**
         * Flag if we have TT&C.
         */
        bool has_ttc;
        /**
         * Main runtime for the TT&C radios.
         */
        Handle<TtcRuntime> ttc_runtime;
        /**
         * Command filter for the non-synced command inputs.
         */
        Handle<CommonCommandFilter> command_filter_nonsynced;
        /**
         * Manages alert telemetry flows.
         */
        Handle<AlertManagerRuntime> alert_mgr_runtime;
        /**
         * The channel that buffered alert telemetry will be written to. The
         * input to our AlertBufferManager.
         */
        Handle<DgramChannel> alert_telem_channel;
        /**
         * Buffers historic telemetry relevant to individual alerts, and sends
         * it over RF if those alerts fire.
         */
        Handle<AlertBufferManager> alert_buffer_manager;
        /**
         * Limits the rate at which we drain buffered alert telemetry to the
         * vehicle network.
         */
        Handle<ByteQuotaFramer> alert_buffer_output;
        /**
         * The interface for interacting with the solar array power converters
         * on this string.
         */
        Handle<DronePowerConverterInterface> sapc_interface;
        /**
         * The interface for the real time clock that persists across boots.
         */
        Handle<RealTimeClockInterface> rtc_interface;
        /**
         * Receives timestamp packets from Swift.
         */
        Handle<UdpConnection> timestamp_channel;
        /**
         * Processes PPS timestamp packets into Slate.
         */
        Handle<SwiftPpsInterface> swift_pps_interface;
        /**
         * Determines the PPS timestamp for this string (from GPS
         * or PNT alternatives) and tracks the vehicle steady clock.
         */
        Handle<PpsManager> pps_manager;
        /**
         * Checks for heartbeat for the current string and pets the watchdog.
         */
        Handle<WatchdogHeartbeat> watchdog_heartbeat;
        /**
         * The satellite revision.
         */
        satellite_rev_t satellite_rev;
        /**
         * The fcpu board revision.
         */
        fcpu_board_rev_t fcpu_board_rev;
        /**
         * Token linked to the control slate used to tell the SlateSyncer
         * whether to enable hotsyncing with only one peer connected.
         *
         * This is currently only provided for testing purposes and should not
         * be used due to the bug identified in TRAC-28869.
         */
        ReadToken<bool> single_peer_hotsync_enabled_tok;
        /**
         * Token linked to the control slate used to tell the SlateSyncer
         * whether to force disable all slate transfer.
         */
        ReadToken<bool> force_disable_transfer_tok;
        /**
         * Navigation time to write into the real time clock (ignored if zero).
         */
        ReadToken<double> nav_time_to_persist_tok;
    };
} /* end namespace Drone */
#endif /* DRONE_FT_RUNTIME_H */