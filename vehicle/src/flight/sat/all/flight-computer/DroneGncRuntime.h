/**
 * @author Nathan DeVries
 * @date   2019/05/17
 */
#ifndef DRONE_GNC_FT_RUNTIME_H
#define DRONE_GNC_FT_RUNTIME_H
#include "src/flight/common/all/FtRuntime.h"
#include "src/flight/sat/all/common/satellite_constants.h"
namespace Drone
{
    class AlertBufferManager;
    class ByteQuotaFramer;
    class CommonCommandFilter;
    /**
     * Master runtime system for a Drone Orbit and Collision
     * Avoidance GNC.
     *
     * This is run in a seprate process because it is slow and could take more
     * than 100ms to execute.
     */
    class DroneGncFtRuntime : public FtRuntime
    {
    public:
        DroneGncFtRuntime(EventLoop &_eloop, const Configs &_configs,
                             const CmdLine &_cmd,
                             Handle<ControlInterface> _control);

    private:
        bool init_runtime_pre_slate_build() override;
        bool init_runtime_post_slate_build() override;
        bool
        create_bootstrapper(Handle<FtBootstrapper> &_bootstrapper) override;
        bool create_command_filters(
            Handle<ExternalCommandTimeFilter> &time_filter,
            Handle<ExternalCommandFilter> &cmd_filter) override;
        bool create_local_telemetry_connections(
            TelemetryRelayRuntime &relay) override;
        bool create_alert_buffer_manager();
        std::string get_telemetry_config_file_name() const override;
        void dispatch_nonsynced_pre_telemetry() override;
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
    };
} /* namespace Drone */
#endif /* DRONE_GNC_FT_RUNTIME_H */