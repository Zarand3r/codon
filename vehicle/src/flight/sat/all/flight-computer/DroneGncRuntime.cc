/**
 * @author Nathan DeVries
 * @date   2019/05/17
 */
#include "src/bullwinkle/all/AlertBufferManager.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/io/AnyDgramConnection.h"
#include "src/bullwinkle/all/io/BwpChannelWriter.h"
#include "src/bullwinkle/all/io/ByteQuotaFramer.h"
#include "src/flight/common/all/ExternalCommandFilterCommon.h"
#include "src/flight/sat/all/flight-computer/DroneGncControl.h"
#include "src/flight/sat/all/flight-computer/DroneGncFtRuntime.h"
namespace Drone
{
    /**
     * Constructor.
     *
     * @param _eloop The EventLoop.
     * @param _configs Use these config files.
     * @param _cmd Read from this command line.
     * @param _control Handle of a control process to run.
     */
    DroneGncFtRuntime::DroneGncFtRuntime(
        EventLoop &_eloop, const Configs &_configs, const CmdLine &_cmd,
        Handle<ControlInterface> _control)
        : FtRuntime(
              _eloop, _configs, _cmd, FtRuntime::triple_string_colloc_ft_node,
              half_duplex /* slate_syncer_duplex_mode */,
              true /* slate_syncer_synchronous_send */,
              true /* clear_input_channels */,
              false /* compute_data_sharing_input_crc */,
              false /* use_sensor_prefixes */, true /* enable_local_ext_cmd */,
              ExternalCommandDispatcher::config_params_t(
                  Satellite::ext_cmd_dedup_capacity_nonsynced,
                  Satellite::ext_cmd_dedup_expiration,
                  Satellite::ext_cmd_proxy_watchdog_timeout,
                  DroneGncControl::satgnc_control_period,
                  Satellite::ext_cmd_proxy_watchdog_enabled),
              _control),
          alert_mgr_runtime(), force_disable_transfer_tok()
    {}
    /**
     * Initialize anything that expects the existence of control components
     * before the slate is build.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::init_runtime_pre_slate_build()
    {
        FswAbortIf(is_init, false);
        /*
         * Initialize the AlertManagerRuntime.
         */
        FswAbortIf(alert_mgr_runtime, false);
        FswAbortIfNot(
            alert_mgr_runtime.assume_ownership(new AlertManagerRuntime()),
            false);
        FswAbortIfNot(
            alert_mgr_runtime->init(configs, alert_info_groups_config_key,
                                    alert_info_alerts_config_key,
                                    slate_control_read_only, *telem_relay),
            false);
        /*
         * Create the alert buffer manager. This needs to bind to control
         * elements created by AlertManagerControl.
         */
        FswAbortIfNot(create_alert_buffer_manager(), false);
        /*
         * Bind to the element in the control slate that determines whether
         * SlateSyncer should enable hotsyncing with only one peer connected.
         *
         * This element is slate-shared over from the satfc control process.
         *
         * This is currently only provided for testing purposes and should not
         * be used due to the bug identified in TRAC-28869.
         */
        FswAbortIfNot(
            slate_control_read_only.bind("satfc1.single_peer_hotsync_enabled",
                                         single_peer_hotsync_enabled_tok),
            false);
        /*
         * Create the element in the control slate that determines whether
         * SlateSyncer should disable transfer.
         */
        FswAbortIfNot(slate_control_sync_only.create(
                          "force_disable_transfer", false, shard_sync,
                          slate_read_write, force_disable_transfer_tok),
                      false);
        return true;
    }
    /**
     * Initialize systems that require slate to have already been built.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::init_runtime_post_slate_build()
    {
        FswAbortIf(is_init, false);
        /*
         * Enable data transfer and hotsync.
         */
        FswAbortIfNot(slate_syncer->enable_transfer(), false);
        FswAbortIfNot(slate_syncer->enable_hotsync(), false);
        FswAbortIfNot(
            alert_mgr_runtime->finalize(smoketest_config, *telem_relay), false);
        return true;
    }
    /**
     * Create the FtBootstrapper.
     *
     * @param[out] _bootstrapper Returns the newly constructed bootstrapper
     *                           object.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::create_bootstrapper(
        Handle<FtBootstrapper> &_bootstrapper)
    {
        const str_v time_sync_bootstrap_nodes(1, ident.role_inst);
        /*
         * Delays need to be a minimum of one cycle:
         *  - auto_time_sync_delay is unused for this process, but must be set.
         *  - Set sharing_enabled_delay to be at least 2 cycles
         *  - Set min_trigger_strings_2_delta_delay to 5 cycles to give plenty
         *    of time for slow booting strings to sync up nominally. Unlike the
         *    main control process, there's no rush here since satgnc doesn't
         *    have a watchdog to pet.
         *  - Set min_trigger_strings_1_delta_delay to at least 2 cycles.
         */
        FtBootstrapper::delay_override_t delays;
        delays.auto_time_sync_delay = DroneGncControl::satgnc_control_period;
        delays.sharing_enabled_delay =
            2 * DroneGncControl::satgnc_control_period;
        delays.min_trigger_strings_2_delta_delay =
            5 * DroneGncControl::satgnc_control_period;
        delays.min_trigger_strings_1_delta_delay =
            2 * DroneGncControl::satgnc_control_period;
        _bootstrapper = FtBootstrapper::create(
            slate_local, slate_control_read_only, ident, control_period,
            time_sync_bootstrap_nodes, FtBootstrapper::no_auto_time_sync,
            FtBootstrapper::reduce_min_trigger_strings,
            FtBootstrapper::establish_input_sync,
            FtBootstrapper::no_output_sync, FtBootstrapper::use_slate_syncer,
            delays);
        FswAbortIfNot(_bootstrapper, false);
        return true;
    }
    /**
     * Create the command filters used by the Satellite local (non-synced)
     * external command system.
     *
     * @param[out] time_filter The time filter to create.
     * @param[out] cmd_filter The command filter to create.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::create_command_filters(
        Handle<ExternalCommandTimeFilter> &time_filter,
        Handle<ExternalCommandFilter> &cmd_filter)
    {
        FswAbortIfNot(
            time_filter.assume_ownership(new ExternalCommandTimeFilterNull),
            false);
        /*
         * Create the command filter.
         *
         * We are re-using the common ExternalCommandFilter, because it is
         * sufficiently configurable through config and isn't really vehicle
         * specific.
         */
        FswAbortIfNot(command_filter_nonsynced.assume_ownership(
                          new CommonCommandFilter("command_filter_nonsynced")),
                      false);
        FswAbortIfNot(
            command_filter_nonsynced->init_storage(
                configs, slate_local, shard_nonsync, true /* enable_at_init */),
            false);
        Handle<ExternalCommandFilterCommon> common_cmd_filter(
            new ExternalCommandFilterCommon);
        FswAbortIfNot(common_cmd_filter->init(command_filter_nonsynced), false);
        cmd_filter = common_cmd_filter;
        return true;
    }
    /**
     * Tell the TelemetryRelayRuntime which services should be directed to local
     * writers instead of sent directly to the network.
     *
     * @param relay The newly-created TelemetryRelayRuntime, before it has been
     *              initialized.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::create_local_telemetry_connections(
        TelemetryRelayRuntime &relay)
    {
        /*
         * Pick a small number to have enough space to hold a couple dgrams in
         * case we have to buffer.
         */
        constexpr size_t num_dgrams = 10;
        FswAbortIfNot(alert_telem_channel.assume_ownership(
                          new DataDgramChannel(num_dgrams, bwp_mtu)),
                      false);
        Handle<BwpChannelWriter> writer;
        FswAbortIfNot(writer.assume_ownership(new BwpChannelWriter()), false);
        FswAbortIfNot(writer->assign_channel(alert_telem_channel), false);
        FswAbortIfNot(relay.redirect_service(
                          Satellite::alert_buffer_input_service, writer),
                      false);
        return true;
    }
    /**
     * Dispatch things that need to be run late in the control cycle - after
     * control code has run but before telemetry is dispatched.
     */
    void DroneGncFtRuntime::dispatch_nonsynced_pre_telemetry()
    {
        const nano_t control_time = eloop.clock.control_time();
        alert_buffer_manager->dispatch(control_time);
        alert_buffer_output->dispatch(control_time);
        /*
         * Set the single peer hotsync enable flag based on the value in the
         * control slate.
         */
        FswIfNot(slate_syncer->set_single_peer_hotsync(
            slate[single_peer_hotsync_enabled_tok]));
        /*
         * Force disable transfer if the flag on the control slate is high.
         */
        if (slate[force_disable_transfer_tok])
        {
            FswIfNot(slate_syncer->disable_transfer());
        }
    }
    /**
     * Create our AlertBufferManager, which buffers historic telemetry relevant
     * to indivdual alerts and sends it over RF if those alerts fire. This
     * needs to be called after the SlateTelemetryTask is created.
     *
     * @return True on success.
     */
    bool DroneGncFtRuntime::create_alert_buffer_manager()
    {
        SlateBuilder alert_buffer_builder =
            slate_local.sub_slate("alert_buffers");
        /*
         * Create our output channel into the alerts buffer.
         */
        Service alerts_service;
        FswAbortIfNot(
            service_directory().lookup(Satellite::alert_buffer_output_service,
                                       alerts_service),
            false);
        FswAbortIf(alerts_service.proto != udp_proto, false);
        Handle<AnyDgramConnection> output_connection;
        FswAbortIfNot(output_connection.assume_ownership(
                          new AnyDgramConnection(upkeep_list, eloop.fds)),
                      false);
        FswAbortIfNot(output_connection->open(alerts_service.host_name,
                                              alerts_service.port),
                      false);
        /*
         * Limit the rate at which we drain data to the vehicle network.
         *
         * Set the bit rate such that we drain at 200kbps, since HOOTLs can't
         * seem to handle much more than this. This happens to match our
         * current minimum downlink rate.
         *
         * Set the quota to a tenth of this (in bytes) so that we downlink at a
         * steady rate when full.
         */
        const UINT64 recharge_bits_per_sec = 200000;
        const UINT64 max_quota = recharge_bits_per_sec / 8 / 10;
        const UINT64 signal_threshold = bwp_mtu;
        FswAbortIfNot(alert_buffer_output.assume_ownership(new ByteQuotaFramer(
                          output_connection, max_quota, recharge_bits_per_sec,
                          signal_threshold)),
                      false);
        FswAbortIfNot(
            alert_buffer_output->init(alert_buffer_builder, "framer_dgrams"),
            false);
        /*
         * Create the buffer manager.
         */
        FswAbortIfNot(alert_buffer_manager = AlertBufferManager::create(
                          configs, "alert_buffers", "alerts",
                          get_telemetry_config_file_name(),
                          Satellite::alert_buffer_input_service, ident,
                          slate_control_read_only,
                          TimestampSynchronizer::synchronized_timestamp_path,
                          alert_buffer_builder, alert_telem_channel,
                          alert_buffer_output,
                          DroneGncControl::satgnc_control_period),
                      false);
        return true;
    }
    /**
     * Get the name of the telemetry config file.
     *
     * @return The name of the telemetry config file.
     */
    std::string DroneGncFtRuntime::get_telemetry_config_file_name() const
    {
        return "telemetry";
    }
} /* namespace Drone */