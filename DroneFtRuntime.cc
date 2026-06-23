/**
 * @author Andy Bohn
 * @date   2018-04-22
 */
#include "src/flight/sat/all/flight-computer/DroneFtRuntime.h"
#include "src/bullwinkle/all/StreamChannelTelemetryTask.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/io/AnyDgramConnection.h"
#include "src/bullwinkle/all/io/BwpChannelWriter.h"
#include "src/bullwinkle/all/io/DataStreamChannel.h"
#include "src/bullwinkle/all/vehicle_network.h"
#include "src/flight/common/all/ExternalCommandFilterCommon.h"
#include "src/flight/sat/all/common/satellite_utils.h"
#include "src/flight/sat/all/flight-computer/FirmwareCommAdcInit.h"
#include "src/flight/sat/all/flight-computer/DroneControl.h"
#include "src/flight/sat/all/power/DroneDistributedPowerConverterInterface.h"
#include "src/flight/sat/all/power/DroneMuxedPowerConverterInterface.h"
namespace Drone
{
    namespace
    {
        /**
         * Path to the spidev device for the AD9361's SPI interface.
         */
        const std::string ad9361_device = "/dev/spidev/ttc";
        /**
         * The name of each GPIO necessary to power the AD9361.
         */
        const str_v ad9361_power_gpios = {{
            "ad9361_pwr_en",
        }};
        /**
         * Path to all FPGA devices in sysfs.
         */
        const std::string fpga_devices = "/sys/devices/platform/amba_pl";
        /**
         * Path to the file containing the FPGA misc register bank.
         */
        const std::string ttc_misc_dir =
            fpga_devices + "/amba_pl:fpga_mmap/mmap";
        /**
         * Path to the directory containing the TT&C control register
         * bank files.
         */
        const std::string ttc_control_dir =
            fpga_devices + "/b8000000.axil_export/mmap";
        /**
         * Offset of the TTC config register in the misc bank, in 32-bit words.
         */
        const size_t ttc_misc_config_offset = 0x19;
    } // namespace
    /**
     * Constructor.
     *
     * @param _eloop The EventLoop.
     * @param _configs Use these config files.
     * @param use_firmware_comm Whether this board uses FirmwareComm.
     * @param _enable_controlcode_static_analysis True to enable full
     *        ControlCode static analysis. False to disable some analysis to
     *        reduce init time.
     */
    DroneFtRuntime::DroneFtRuntime(
        EventLoop &_eloop, const Configs &_configs,
        const Cbor::Entry &_manifest_info, const bool _has_ttc,
        const bool use_firmware_comm, const CmdLine &_cmd,
        const bool _enable_controlcode_static_analysis)
        : FtRuntime(_eloop, _configs, _cmd,
                    (use_firmware_comm)
                        ? FtRuntime::triple_string_ft_node
                        : FtRuntime::triple_string_guest_ft_node,
                    sixth_duplex /* slate_syncer_duplex_mode */,
                    true /* slate_syncer_synchronous_send */,
                    true /* clear_input_channels */,
                    false /* compute_data_sharing_input_crc */,
                    false, /* use_sensor_prefixes */
                    true /* enable_local_ext_cmd */,
                    ExternalCommandDispatcher::config_params_t(
                        Satellite::ext_cmd_dedup_capacity_nonsynced,
                        Satellite::ext_cmd_dedup_expiration,
                        Satellite::ext_cmd_proxy_watchdog_timeout,
                        Satellite::ext_cmd_proxy_watchdog_interval,
                        Satellite::ext_cmd_proxy_watchdog_enabled),
                    Handle<ControlInterface>(new DroneControl(
                        _cmd.get_string("text_pattern_config"),
                        _enable_controlcode_static_analysis,
                        create_init_only(alert_info_init_only), _manifest_info,
                        _has_ttc))),
          manifest_info(_manifest_info), has_ttc(_has_ttc), ttc_runtime(),
          command_filter_nonsynced(), alert_mgr_runtime(),
          alert_telem_channel(), alert_buffer_manager(), alert_buffer_output(),
          sapc_interface(), timestamp_channel(), swift_pps_interface(),
          pps_manager(), watchdog_heartbeat(),
          satellite_rev(satellite_rev_undefined),
          fcpu_board_rev(fcpu_board_rev_undefined), force_disable_transfer_tok()
    {}
    /**
     * Destructor.
     */
    DroneFtRuntime::~DroneFtRuntime() {}
    /**
     * Perform initialization of Satellite-specific systems that must exist
     * prior to initializing the control components.
     *
     * @return True on success.
     */
    bool DroneFtRuntime::init_runtime_pre_control()
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(Satellite::get_satellite_rev(configs, satellite_rev),
                      false);
        FswAbortIfNot(Satellite::get_fcpu_board_rev(configs, fcpu_board_rev),
                      false);
        /*
         * Set the initial values for FirmwareComm outputs
         * if this board uses FirmwareComm.
         */
        if (props.use_firmware_comm)
        {
            FswAbortIfNot(FirmwareCommAdcInit::init_hardware(
                              configs, "firmwarecomm_adc_init"),
                          false);
        }
        return true;
    }
    /**
     * Perform pre-slate-build initialization of Satellite-specific systems
     * that may rely on the existence of control components.
     *
     * @return True on success.
     */
    bool DroneFtRuntime::init_runtime_pre_slate_build()
    {
        FswAbortIf(is_init, false);
        /*
         * Create the TT&C radio system, SAPC Interface, and Real Time Clock
         * interface on satfcs
         */
        if (ident.role == "satfc")
        {
            /*
             * On board revisions using FirmwareComm, the 'heartbeat_x' slate
             * elements are mapped as FirmwareComm DOs (digital outputs) and
             * exposed by the FPGA as PL GPIOs.
             *
             * On board revisions that _don't_ use FirmwareComm, we repurpose
             * the 'heartbeat_x' slate elements to pet the Linux watchdog device
             * (/dev/watchdog), which is connected (via EMIO) to the FPGA and
             * drives the same PL GPIOs.
             */
            if (!props.use_firmware_comm)
            {
                FswAbortIfNot(watchdog_heartbeat = WatchdogHeartbeat::create(
                                  slate_control_read_only, ident),
                              false);
            }
            if (has_ttc)
            {
                FswAbortIfNot(create_ttc_radio_system(), false);
            }
            bool has_distributed_sapcs;
            bool has_muxed_sapcs;
            has_distributed_sapcs = (fcpu_board_rev == fcpu_board_rev_1 ||
                                     fcpu_board_rev == fcpu_board_rev_3 ||
                                     fcpu_board_rev == fcpu_board_rev_4);
            has_muxed_sapcs = (fcpu_board_rev == fcpu_board_rev_5);
            size_t num_sapcs;
            if (has_distributed_sapcs)
            {
                num_sapcs = Drone::num_sapcs_per_fc_distributed;
            }
            else if (has_muxed_sapcs)
            {
                num_sapcs = Drone::num_sapcs_per_fc_muxed;
            }
            else
            {
                num_sapcs = 0;
            }
            /*
             * Set up one StreamChannel for each power converter instance to
             * telemeter high speed telem to.
             */
            std::vector<Handle<StreamChannel>> sapc_telem_channels;
            /*
             * Allow a 2x overhead in each stream channel.
             */
            constexpr size_t data_stream_channel_buffer =
                2 * DronePowerConverterInterface::sweep_telem_size;
            for (size_t i = 0; i < num_sapcs; i++)
            {
                sapc_telem_channels.push_back(Handle<DataStreamChannel>(
                    new DataStreamChannel(data_stream_channel_buffer)));
            }
            /*
             * Create the power converter interface which binds to tokens
             * created by the power converter controller in the synced logic.
             */
            FswAbortIf(sapc_interface, false);
            if (has_distributed_sapcs)
            {
                FswAbortIfNot(
                    sapc_interface =
                        DroneDistributedPowerConverterInterface::create(
                            slate_control_read_only, slate_local, ident,
                            sapc_telem_channels),
                    false);
            }
            else if (has_muxed_sapcs)
            {
                FswAbortIfNot(sapc_interface =
                                  DroneMuxedPowerConverterInterface::create(
                                      slate_control_read_only, slate_local,
                                      ident, sapc_telem_channels),
                              false);
            }
            bool has_gnss = false;
            FswAbortIfNot(
                get_cbor_data("satfc.has_gnss", manifest_info, has_gnss),
                false);
            /*
             * Initialize PPS components.
             */
            if (has_gnss)
            {
                /*
                 * Open a channel to receive timestamp packets from Swift.
                 */
                FswAbortIfNot(timestamp_channel.assume_ownership(
                                  new UdpConnection(upkeep_list, eloop.fds)),
                              false);
                {
                    Service service;
                    const std::string timestamp_service =
                        "satgps1" + ident.string + "_timestamp";
                    FswAbortIfNot(
                        service_directory().lookup(timestamp_service, service),
                        false);
                    FswAbortIfNot(service.proto == udp_proto, false);
                    /*
                     * SATSW-81271: don't open the timestamp channel until
                     * satellite_ptp_driver is no longer listening to the same
                     * service.
                     */
                    // FswAbortIfNot(timestamp_channel->open(service.port),
                    // false);
                }
                /*
                 * Initialize the Swift PPS timestamp parser.
                 */
                SlateBuilder swift_pps_subslate =
                    slate_local.sub_slate("swift_pps_interface");
                FswAbortIfNot(swift_pps_interface.assume_ownership(
                                  new SwiftPpsInterface()),
                              false);
                FswAbortIfNot(
                    swift_pps_interface->init(swift_pps_subslate,
                                              timestamp_channel->read_sig),
                    false);
                /*
                 * Initialize the PPS manager.
                 */
                FswAbortIfNot(pps_manager = PpsManager::create(
                                  slate_local, slate_control_read_only,
                                  swift_pps_subslate, ident),
                              false);
            }
            /*
             * Add the SAPC telemetry producers. These two flows are the only
             * flows in this telemetry group, so we know the flow indices should
             * be 0 and 1 respectively.
             */
            for (size_t i = 0; i < num_sapcs; i++)
            {
                Handle<StreamChannelTelemetryTask> telem_task(
                    new StreamChannelTelemetryTask(eloop.clock));
                FswAbortIfNot(telem_task, false);
                FswAbortIfNot(telem_task->init(*telem_relay,
                                               telem_group_sat_sapc,
                                               i, /* flow index */
                                               sapc_telem_channels[i]),
                              false);
                FswAbortIfNot(telem_relay->add_producer(telem_task), false);
            }
            /*
             * Initialize the real time clock interface.
             */
            FswAbortIfNot(rtc_interface = RealTimeClockInterface::create(
                              slate_local, ident, "/dev/rtc0"),
                          false);
            /*
             * Bind to the control token used to update the real time clock.
             */
            FswAbortIfNot(
                slate_control_read_only.bind("rtc.nav_time_to_persist",
                                             nav_time_to_persist_tok),
                false);
        }
        /*
         * Initialize the AlertManagerRuntime.
         */
        FswAbortIf(alert_mgr_runtime, false);
        FswAbortIfNot(
            alert_mgr_runtime.assume_ownership(new AlertManagerRuntime()),
            false);
        FswAbortIfNot(
            alert_mgr_runtime->init(configs, alert_info_init_only->group_infos,
                                    alert_info_init_only->alert_infos,
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
         */
        FswAbortIfNot(
            slate_control_read_only.bind("single_peer_hotsync_enabled",
                                         single_peer_hotsync_enabled_tok),
            false);
        /*
         * Bind to the element in the control slate that determines whether
         * SlateSyncer should disable transfer.
         */
        FswAbortIfNot(slate_control_sync_only.bind("force_disable_transfer",
                                                   force_disable_transfer_tok),
                      false);
        return true;
    }
    /**
     * Initialize Satellite-specific systems that require slate to have already
     * been built.
     *
     * @return True on success.
     */
    bool DroneFtRuntime::init_runtime_post_slate_build()
    {
        FswAbortIf(is_init, false);
        /*
         * Enable data transfer and hotsync.
         */
        FswAbortIfNot(slate_syncer->enable_transfer(), false);
        FswAbortIfNot(slate_syncer->enable_hotsync(), false);
        /*
         * Make sure every IP in the node directory is inside a satellite
         * network segment.
         */
        std::vector<vehicle_network_segment_t> segments;
        segments.push_back(vehicle_network_segment_ground);
        segments.push_back(vehicle_network_segment_rf);
        segments.push_back(vehicle_network_segment_satellite);
        segments.push_back(vehicle_network_segment_satellite_utility);
        segments.push_back(vehicle_network_segment_satellite_payload);
        FswAbortIfNot(verify_node_directory_segments(node_directory(), segments,
                                                     true /* allow_192_168 */),
                      false);
        /*
         * Initialize the command filter.
         * Note that the non-synced command platform does not have access to a
         * state machine or alert provider.
         */
        CommandTable empty_cmd_table;
        FswAbortIfNot(empty_cmd_table.init(), false);
        const str_v empty_sm_prefixes;
        FswAbortIfNot(command_filter_nonsynced, false);
        FswAbortIfNot(command_filter_nonsynced->init(configs, slate_local,
                                                     empty_cmd_table,
                                                     empty_sm_prefixes),
                      false);
        /*
         * Finalize the AlertManagerRuntime.
         */
        FswAbortIfNot(
            alert_mgr_runtime->finalize(smoketest_config, *telem_relay,
                                        alert_info_init_only->alert_infos),
            false);
        /*
         * Release the init-only data to both save memory and make sure it is
         * not used at runtime.
         */
        decltype(alert_info_init_only)::weak_type weak_init_only =
            alert_info_init_only;
        alert_info_init_only.reset();
        FswAbortIfNot(weak_init_only.expired(), false);
        return true;
    }
    /**
     * Dispatch things that need to be run early in the control cycle - before
     * the control code has run and before FirmwareComm outputs are written.
     */
    void DroneFtRuntime::dispatch_nonsynced_pre_firmware_comm()
    {
        if (sapc_interface)
        {
            FswIfNot(sapc_interface->dispatch());
        }
        if (watchdog_heartbeat)
        {
            watchdog_heartbeat->dispatch();
        }
    }
    /**
     * Dispatch things that need to be run late in the control cycle - after
     * control code has run but before telemetry is dispatched.
     */
    void DroneFtRuntime::dispatch_nonsynced_pre_telemetry()
    {
        const nano_t control_time = eloop.clock.control_time();
        alert_buffer_manager->dispatch(control_time);
        alert_buffer_output->dispatch(control_time);
        /*
         * Drone 1p5+ does not have a ttc_runtime, but otherwise update the
         * TT&C hardware configuration.
         */
        if (ttc_runtime)
        {
            ttc_runtime->dispatch();
        }
        /*
         * Dispatch the PPS manager.
         */
        if (pps_manager)
        {
            FswIfNot(pps_manager->dispatch());
        }
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
        /*
         * Dispatch the real time clock interface, then update the real time
         * clock from this cycle's navigation time.
         */
        if (rtc_interface)
        {
            FswIfNot(rtc_interface->dispatch());
            if (slate[nav_time_to_persist_tok] > 0)
            {
                const nano_t update_unix_time = gpstime_gps_to_unix(
                    leap_second_table(),
                    static_cast<nano_t>(slate[nav_time_to_persist_tok] *
                                        billion));
                FswIfNot(
                    rtc_interface->update_rtc_from_unix_time(update_unix_time));
            }
        }
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
    bool DroneFtRuntime::create_local_telemetry_connections(
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
        /*
         * BwpChannelWriter::assign_channel() is a bit presumptuous,
         * opting us into a "pull" I/O model that we don't need.
         *
         * Telemetry producers "push" all their datagrams into the
         * output channel when they are dispatched;
         * there's no need to ask them for more data later that cycle.
         *
         * Opt out of the "pull" model by disconnecting the writer
         * from the output channel's write signal.
         *
         * The writer is the only subscriber so it's fine to call clear() here;
         * there's not a convenient API to disconnect a single subscriber,
         * anyway.
         */
        alert_telem_channel->write_sig.clear();
        FswAbortIfNot(relay.redirect_service(
                          Satellite::alert_buffer_input_service, writer),
                      false);
        return true;
    }
    /**
     * Tells the Telemetry Relay to create only one connection per telemetry
     * destination. Doing so can save a substantial number of file descriptors.
     */
    bool
    DroneFtRuntime::should_telem_relay_aggregate_dest_connections() const
    {
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
    bool DroneFtRuntime::create_bootstrapper(
        Handle<FtBootstrapper> &_bootstrapper)
    {
        /*
         * The satfc1 node should always be the time synchronization initiator.
         */
        const str_v time_sync_bootstrap_nodes(1, "satfc1");
        FtBootstrapper::time_sync_mode_t time_sync_mode =
            FtBootstrapper::auto_time_sync;
        /*
         * We decrease the delay before lowering to 2 strings for input sync to
         * make sure a system with one string down can get synced and start
         * petting the watchdog in a reasonable amount of time.
         *
         * Never drop down to 1 string. One Drone FCPU string cannot do much
         * on its own, e.g. it cannot actuate load switches without another
         * string due to hardware voting. Wait for 2 strings until we are
         * watchdogged. We set the value to 1 day, which is much longer than a
         * FC watchdog.
         *
         * Note: it's not a huge deal if the third string doesn't sync here.  It
         * will just hotsync in later when it does come up.
         */
        FtBootstrapper::delay_override_t delays;
        delays.min_trigger_strings_2_delta_delay = to_nano_t(10.0);
        delays.min_trigger_strings_1_delta_delay = to_nano_t(gnc_sec_p_day);
        _bootstrapper = FtBootstrapper::create(
            slate_local, slate_control_read_only, ident, control_period,
            time_sync_bootstrap_nodes, time_sync_mode,
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
    bool DroneFtRuntime::create_command_filters(
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
     * Setup all the hardware and software necessary to use the TT&C radios.
     *
     * @return True on success.
     */
    bool DroneFtRuntime::create_ttc_radio_system()
    {
        FswAbortIf(is_init, false);
        const str_v real_radio_nodes = Satellite::get_ttc_strings(configs);
        const bool real_radio =
            (std::find(real_radio_nodes.begin(), real_radio_nodes.end(),
                       ident.node_name) != real_radio_nodes.end());
        /*
         * Initialize TT&C hardware. Some of this logic binds to slate elements
         * created in TtcControl.
         */
        FswAbortIfNot(ttc_runtime = TtcRuntime::create(
                          eloop, upkeep_list, ident, service_directory(),
                          configs, slate_control_read_only, slate_local,
                          ad9361_device, ttc_misc_dir, ttc_misc_config_offset,
                          ttc_control_dir, real_radio, ad9361_power_gpios),
                      false);
        return true;
    }
    /**
     * Create our AlertBufferManager, which buffers historic telemetry relevant
     * to indivdual alerts and sends it over RF if those alerts fire. This
     * needs to be called after the SlateTelemetryTask is created.
     *
     * @return True on success.
     */
    bool DroneFtRuntime::create_alert_buffer_manager()
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
                          alert_buffer_output, control_period),
                      false);
        return true;
    }
    /**
     * Get the name of the telemetry config file. For the satellite the config
     * file differs based on the flight computer string. Each file includes a
     * common telemetry config file.
     *
     * @return The name of the telemetry config file.
     */
    std::string DroneFtRuntime::get_telemetry_config_file_name() const
    {
        return "telemetry_" + ident.string;
    }
} /* end namespace Drone */