/**
 * @author Steve Gerding
 * @date   2015-03-10
 */
#ifndef FT_RUNTIME_H
#define FT_RUNTIME_H
#include "src/bullwinkle/all/AdcScaler.h"
#include "src/bullwinkle/all/AlertManagerRuntime.h"
#include "src/bullwinkle/all/CmdLine.h"
#include "src/bullwinkle/all/CommandSpammer.h"
#include "src/bullwinkle/all/Configs.h"
#include "src/bullwinkle/all/CycleTimer.h"
#include "src/bullwinkle/all/DeviceTelemetryFactory.h"
#include "src/bullwinkle/all/EnumRegistry.h"
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/EventLoop.h"
#include "src/bullwinkle/all/ExternalCommandDispatcher.h"
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/FirmwareComm.h"
#include "src/bullwinkle/all/FtraceTrap.h"
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Keychain.h"
#include "src/bullwinkle/all/NodeIdentifier.h"
#include "src/bullwinkle/all/ReflectionManager.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateAlarmTask.h"
#include "src/bullwinkle/all/SlateCombiner.h"
#include "src/bullwinkle/all/SlateCommandInterface.h"
#include "src/bullwinkle/all/SlateSharer.h"
#include "src/bullwinkle/all/SlateSyncer.h"
#include "src/bullwinkle/all/SlateTelemetryTask.h"
#include "src/bullwinkle/all/SlateTrap.h"
#include "src/bullwinkle/all/StateMachine.h"
#include "src/bullwinkle/all/TelemetryRelay.h"
#include "src/bullwinkle/all/TimeScaler.h"
#include "src/bullwinkle/all/ft/FtChannelManager.h"
#include "src/bullwinkle/all/ft/FtNodeConfig.h"
#include "src/bullwinkle/all/identity_util.h"
#include "src/bullwinkle/all/io/BwpFramer.h"
#include "src/bullwinkle/all/io/DgramChannel.h"
#include "src/bullwinkle/all/slate_dump/SlateDump.h"
#include "src/dna/all/dna_logger.h"
#include "src/flight/common/all/ControlInterface.h"
#include "src/flight/common/all/FtBootstrapper.h"
#include "src/flight/common/all/FtSimpleDataSharer.h"
#include "src/flight/common/all/FtSync.h"
#include "src/flight/common/all/NodeIoManager.h"
#include "src/flight/common/all/TimestampSynchronizer.h"
#include "src/gnc/all/GncController.h"
#include <string>
#include <vector>
namespace Drone
{
    /**
     * A class that wraps a ControlInterface object and allows it to run in
     * the context of a triple-string node by providing I/O facilities and
     * synchronization.
     */
    class FtRuntime : public SignalHandler
    {
    public:
        /**
         * An enumeration of the different node types that FtRuntime supports.
         */
        enum ft_node_type_t
        {
            /**
             * A vanilla triple-string node that is not part of a triple-dual
             * system or collocated on the same physical computer as another
             * control process.
             */
            triple_string_ft_node,
            /**
             * The master side of a triple-dual system.
             */
            triple_dual_master_ft_node,
            /**
             * The slave side of a triple-dual system.
             */
            triple_dual_slave_ft_node,
            /**
             * A non-triple-dual triple-string that is collocated on the same
             * physical computer as another control process.
             */
            triple_string_colloc_ft_node,
            /**
             * A triple-string node that is not part of a triple-dual
             * system, but can be collocated wth other processes on the same
             * physical computer. This process does not run FirmwareComm
             * and can be imagined as a "Virtual Machine" on a host,
             * that doesn't particularly care about which host it is on.
             */
            triple_string_guest_ft_node
        };
        FtRuntime(
            EventLoop &_eloop, const Configs &_configs, const CmdLine &_cmd,
            const ft_node_type_t ft_node_type,
            const slate_syncer_duplex_mode_t _slate_syncer_duplex_mode,
            const bool _slate_syncer_synchronous_send,
            const bool _clear_input_channels,
            const bool _compute_data_sharing_input_crc,
            const bool _use_sensor_prefixes, const bool _enable_local_ext_cmd,
            const ExternalCommandDispatcher::config_params_t &_ext_cmd_params,
            Handle<ControlInterface> _control);
        virtual ~FtRuntime();
        bool init();
        bool dump_devices();
        nano_t dispatch(nano_t);

    protected:
        /**
         * A struct that demultiplexes a ft_node_type_t into a set of useful
         * properties for configuring the FtRuntime object.
         */
        struct FtNodeProperties
        {
        public:
            /**
             * Construct the FtNodeProperties object by setting the properties
             * based on the node type.
             *
             * @param ft_node_type The node type.
             */
            explicit FtNodeProperties(const ft_node_type_t ft_node_type)
                : use_input_sync(ft_node_type != triple_dual_slave_ft_node),
                  use_output_sync(ft_node_type == triple_dual_master_ft_node ||
                                  ft_node_type == triple_dual_slave_ft_node),
                  use_slate_syncer(ft_node_type != triple_dual_master_ft_node &&
                                   ft_node_type != triple_dual_slave_ft_node),
                  use_firmware_comm(ft_node_type == triple_string_ft_node ||
                                    ft_node_type ==
                                        triple_string_colloc_ft_node),
                  use_ftrace_trap(ft_node_type != triple_dual_slave_ft_node &&
                                  ft_node_type != triple_string_colloc_ft_node)
            {}
            /**
             * Whether this process will use NodeIoManager and
             * FtSimpleDataSharer to read synchronized inputs.
             *
             * All node types except the triple-dual slave participate in input
             * synchronization.
             */
            const bool use_input_sync;
            /**
             * Whether this process will make use of output synchronization. If
             * true, we will not connect the channel manager up to the data
             * sharer and node IO manager so we can instead hook it up to the
             * output synchronization system.
             *
             * Only the triple-dual master and slave participate in output
             * synchronization.
             */
            const bool use_output_sync;
            /**
             * Whether this process will use SlateSyncer to monitor for desyncs
             * and/or hotsync state from its peers.
             */
            const bool use_slate_syncer;
            /**
             * Whether this process will use FirmwareComm to communicate with
             * hardware devices.
             *
             * Most non-triple-dual nodes utilize FirmwareComm, however
             * (at time of writing) Starship allows for control processes that
             * interact over the network only, without FirmwareComm.
             */
            const bool use_firmware_comm;
            /**
             * Whether this process will instantiate and use an FtraceTrap to
             * create ftrace snapshots based on multi-sensor alarms.
             *
             * It is only useful to have a single FtraceTrap per physical
             * computer, so triple-dual slaves and collocated triple-string
             * nodes do not instantiate it.
             */
            const bool use_ftrace_trap;
        };
        /*
         * Top-level initialization phases.
         */
        bool init_pre_control_early();
        bool init_pre_control_late();
        bool init_pre_slate_build_early();
        bool init_pre_slate_build_late();
        bool init_post_slate_build_early();
        bool init_post_slate_build_late();
        /*
         * Vehicle-specific initialization phases.
         */
        virtual bool init_runtime_pre_control();
        virtual bool init_runtime_pre_slate_build();
        virtual bool init_runtime_post_slate_build();
        /*
         * Top-level control cycle phases.
         */
        void dispatch_nonsynced_early();
        void dispatch_nonsynced_late();
        /*
         * Vehicle-specific control cycle phases.
         */
        virtual void dispatch_nonsynced_pre_firmware_comm();
        virtual void dispatch_nonsynced_pre_data_sharing();
        virtual void dispatch_nonsynced_post_data_sharing();
        virtual void dispatch_nonsynced_pre_telemetry();
        /*
         * Utility functions that may vary from vehicle to vehicle. Used
         * only when generic constructions are impossible.
         */
        virtual std::string get_telemetry_config_file_name() const;
        virtual std::set<telem_group_t> get_muxed_telemetry_groups() const;
        virtual int get_control_inst() const;
        virtual size_t get_num_local_reflect_slots() const;
        virtual bool should_telem_relay_aggregate_dest_connections() const;
        virtual bool
        create_local_telemetry_connections(TelemetryRelayRuntime &relay);
        virtual bool
        create_bootstrapper(Handle<FtBootstrapper> &_bootstrapper) = 0;
        virtual bool
        create_command_filters(Handle<ExternalCommandTimeFilter> &time_filter,
                               Handle<ExternalCommandFilter> &cmd_filter) = 0;
        /*
         * Component initialization functions.
         */
        bool create_input_sharing_system(const nano_t share_time,
                                         const nano_t reshare_time);
        bool create_adc_system();
        bool create_bootstrapper_system();
        bool create_nonsynced_command_platform();
        /*
         * Class initialization convenience functions.
         */
        bool create_adc_scaler(SlateBuilder raw, SlateBuilder scaled,
                               const bool scale_ad_banks,
                               Handle<AdcScaler> &scaler);
        std::string local_slate_name() const;
        std::string control_slate_name() const;
        virtual std::string shared_slate_name(const char string) const;
        virtual std::string median_slate_name() const;
        std::string local_enum_prefix() const;
        virtual bool should_overlap_reshare_and_input_writes() const;
        virtual bool should_send_syncs_during_reshare_phase() const;
        /*
         * Event handlers.
         */
        bool handle_local_share_read(DgramChannel &channel, node_id_t node_id);
        bool reset_counters() RUNTIME;
        virtual void reset_runtime_counters() RUNTIME;
        /**
         * The runtime configuration properties for this node.
         */
        const FtNodeProperties props;
        /**
         * The duplex mode SlateSyncer should operate with.
         */
        const slate_syncer_duplex_mode_t slate_syncer_duplex_mode;
        /**
         * If true, FtRuntime will attempt to dispatch SlateSyncer transmit
         * events immediately after the outgoing sync shard is prepared, instead
         * of waiting until the slack time at the end of the cycle. In practice,
         * this allows starting the sync shard transfer (which may be lengthy)
         * as early as possible, and may parallelize it with emission of normal
         * outputs if there is sufficient overall CPU and I/O bandwidth.
         */
        const bool slate_syncer_synchronous_send;
        /**
         * Whether we should clear data in all input channels at the end
         * of the control cycle.
         */
        const bool clear_input_channels;
        /**
         * True to compute the data sharing input_crc each cycle. False to skip
         * this computation and leave the input_crc at zero.
         */
        const bool compute_data_sharing_input_crc;
        /**
         * True to enable sensor prefixing based on config files. Used to allow
         * generic, locally named sensor channels in calibration files and
         * prefix them to be fully specified names when parsed.
         * For example:
         *  engine_sn3.sensor:  mcc-p 100 1
         *  engine_sn7.sensor:  mcc-p 100 1
         *  sensor_prefixes:    eng1 engine_sn3.sensor
         *                      eng2 engine_sn6.sensor
         *  result: eng1.mcc-p and eng2.mcc-p
         */
        const bool use_sensor_prefixes;
        /**
         * True to set up the local (non-synchronized) external commanding
         * system.
         */
        const bool enable_local_external_commanding;
        /**
         * The set of parameters used to configure the local (non-synchronized)
         * ExternalCommandDispatcher, if enabled per the flag above.
         */
        const ExternalCommandDispatcher::config_params_t ext_cmd_params;
        /**
         * The EventLoop. This maintains the control cycle dispatch order
         * and distributes the current time.
         */
        EventLoop &eloop;
        /**
         * Slush list for general upkeep tasks like reconnectors.
         */
        EventList upkeep_list;
        /**
         * The config-file finding object.
         */
        const Configs &configs;
        /**
         * The command line inputs.
         */
        const CmdLine &cmd;
        /**
         * The SmoketestConfig object.
         */
        const SmoketestConfig smoketest_config;
        /**
         * The control logic that this runtime class is wrapping.
         */
        Handle<ControlInterface> control;
        /**
         * True when initialized.
         */
        bool is_init;
        /**
         * The period at which we control.
         */
        nano_t control_period;
        /**
         * The amount of time that systems that do not use input sync will
         * sleep for instead of performing data sharing to keep them in phase
         * with their counterparts that do. Equal to the total duration of
         * share time and reshare time.
         */
        nano_t ds_parallel_sleep_time;
        /**
         * Our identity.
         */
        NodeIdentity ident;
        /**
         * The node configs for the fault-tolerant computing system.
         */
        FtNodeConfigList node_configs;
        /**
         * The root of the slate tree.
         */
        SlateBuilder slate_root;
        /**
         * The root of the local slate.
         */
        SlateBuilder slate_local;
        /*
         * The slate for runtime components which do create elements in the
         * control slate. This will eventually need to be removed, tracked by
         * #14879.
         */
        SlateBuilder slate_control_read_create;
        /**
         * The slate for the control instance.
         */
        SlateBuilder slate_control_sync_only;
        /**
         * A read-only version of the control slate, for use by non-
         * synchronized components that need to read items in the control
         * slate.
         */
        SlateBuilder slate_control_read_only;
        /**
         * The run-time Slate.
         */
        INFRASTRUCTURE(Slate) slate;
        /**
         * {@ @name Timers to keep track of the amount of time elapsed in
         *          various phases of the control cycle.
         */
        CycleTimer slate_syncer_replace_timer;
        CycleTimer firmware_comm_timer;
        CycleTimer pre_data_sharing_timer;
        CycleTimer data_sharing_timer;
        CycleTimer post_data_sharing_timer;
        CycleTimer pre_telemetry_timer;
        CycleTimer telemetry_timer;
        CycleTimer slate_syncer_share_timer;
        CycleTimer slate_roll_frame_timer;
        CycleTimer channel_manager_flush_timer;
        CycleTimer eloop_dispatch_select_timer;
        /**
         * @}
         */
        /**
         * A simple state machine that used to establish synchronization with
         * the other nodes in the system.
         */
        Handle<FtBootstrapper> bootstrapper;
        /**
         * The phase-synchronization system. This keeps all redundant
         * strings of the controller in a phase-locked loop.
         */
        Handle<FtSync> ft_sync;
        /**
         * Keychain used for signed messages.
         */
        Handle<Keychain> keychain;
        /**
         * The channel manager, which handles all fault-tolerant input
         * and output data.
         */
        Handle<FtChannelManager> channel_manager;
        /**
         * The node io manager, which creates the network connections to
         * external services that will eventually populate the channels
         * in channel_manager.
         */
        Handle<NodeIoManager> node_io_manager;
        /**
         * The data sharer, which exchanges input data between strings and
         * populates the channel manager.
         */
        Handle<FtSimpleDataSharer> data_sharer;
        /**
         * Trap which executes when commanded to clear error counters.
         */
        Handle<SlateFlagTrap> reset_counters_trap;
        /**
         * The parsed-out hardware configuration.
         */
        adc_board_v adc_boards_info;
        /**
         * The ADC hardware communication system. This communicates with
         * the FPGA to actuate electrical outputs and read in sensor data.
         */
        Handle<FirmwareComm> firmware_comm;
        /**
         * The hardware scaler for the local space.
         */
        Handle<AdcScaler> adc_scaler_local;
        /**
         * The slate sender to our peers.
         */
        Handle<SlateSharerSender> slate_sender_local;
        /**
         * Slate sync system.
         */
        Handle<SlateSyncer> slate_syncer;
        /**
         * Dedicated FdBag for SlateSyncer file descriptors.
         */
        Handle<FdBag> slate_syncer_fd_bag;
        /**
         * Slate command interface for local elements.
         */
        Handle<SlateCommandInterface> slate_command_interface_local;
        /**
         * Allows runtime-configurable reflection of local state.
         */
        Handle<ReflectionManager> reflection_manager_local;
        /**
         * External command dispatcher for the non-synced command input.
         */
        Handle<ExternalCommandDispatcher> gnd_cmd_dispatcher_nonsynced;
        /**
         * The set of enums to be included in telemetry metadata.
         */
        Handle<EnumRegistry> enum_registry;
        /**
         * The set of scalings to be included in telemetry metadata.
         */
        Handle<scale_info_v> scale_info;
        /**
         * Telemetry channel names which may only be telemetered as scaled
         * values, not as raw values.
         */
        str_s scaled_only_telemetry_channels;
        /**
         * Telemetry system.
         */
        Handle<TelemetryRelayRuntime> telem_relay;
        /**
         * Slate-share the local telemetry timestamp for use by the
         * TimestampSynchronizer.
         */
        Handle<TimestampGatherer> timestamp_gatherer;
        /**
         * Telemetry producer.
         */
        Handle<SlateTelemetryTask> slate_telem;
        /**
         * The null_int device. This token is used to obtain the address of the
         * device so it can be telemetered and ultimately used for desync
         * testing.
         */
        ReadToken<INT64> null_int_tok;
        /**
         * The address of the null_int device, used for desync testing.
         */
        WriteToken<UINT64> null_int_address_tok;
        /**
         * The amount by which to artificaly extend the control cycle. This
         * logic will only take effect if the corresponding "enable" device is
         * true.
         *
         * WARNING: This device is meant to be used during testing ONLY. In
         *          flight, it will be placed behind a command filter (along
         *          with the "enable" device) and should not be commanded.
         */
        ReadToken<double> simulated_control_cycle_extension_tok;
        /**
         * Whether to act on the sleep time specified in the
         * "simulated_control_cycle_extension" device.
         *
         * WARNING: This device is meant to be used during testing ONLY. In
         *          flight, it will be placed behind a command filter (along
         *          with the "extension" device) and should not be commanded.
         */
        ReadToken<bool> enable_simulated_control_cycle_extension_tok;
        /**
         * Current heap usage since initialization ended, in bytes.
         */
        WriteToken<size_t> heap_used_tok;
        /**
         * The maximum heap used since initialization ended, in bytes.
         */
        WriteToken<size_t> heap_max_used_tok;
        /**
         * Cumulative number of allocations since initialization ended.
         */
        WriteToken<size_t> heap_allocations_count_tok;
        /**
         * Periodic to display heap allocation locations. Set to do so every 5m.
         */
        Periodic display_heap_allocation_locations_periodic;
        /**
         * Trap which creates an ftrace snapshot on a user-configured alarm.
         */
        Handle<FtraceTrap> ftrace_trap;
        /**
         * Like ftrace_trap, but traps on only the local string (as opposed to
         * all strings).
         */
        Handle<FtraceTrap> ftrace_trap_local;
        /**
         * The DNA logger.
         */
        Dna::Logger dna;
        /**
         * Slate dumper.
         */
        Handle<SlateDump> slate_dump;
        /**
         * Time scaler.
         */
        TimeScaler time_scaler;
    };
} /* end namespace Drone */
#endif /* FT_RUNTIME_H */