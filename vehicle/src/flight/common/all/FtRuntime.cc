/**
 * @author Steve Gerding
 * @date   2015-03-10
 */
#include "src/flight/common/all/FtRuntime.h"
#include "src/bullwinkle/all/DgramChannelTelemetryTask.h"
#include "src/bullwinkle/all/ExternalCommandReflectionHandler.h"
#include "src/bullwinkle/all/ExternalCommandSlateHandler.h"
#include "src/bullwinkle/all/ExternalCommandStateMachineHandler.h"
#include "src/bullwinkle/all/InitSeed.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/config_file.h"
#include "src/bullwinkle/all/external_command_util.h"
#include "src/bullwinkle/all/ftrace_util.h"
#include "src/bullwinkle/all/heap_tracker.h"
#include "src/bullwinkle/all/io/AnyDgramConnection.h"
#include "src/bullwinkle/all/io/DataDgramChannel.h"
#include "src/bullwinkle/all/io/NullDgramChannel.h"
#include "src/bullwinkle/all/io/TcpConnection.h"
#include "src/bullwinkle/all/io/TcpP2PServerConnection.h"
#include "src/bullwinkle/all/multicast_utils.h"
#include "src/bullwinkle/all/slate_info.h"
#include "src/bullwinkle/all/time_slave_preload.h"
#include "src/flight/common/all/TripleString.h"
#include "src/gnc/all/GncComponentFactory.h"
#include <algorithm>
#include <set>
#include <utility>
namespace Drone
{
    /**
     * Constructor.
     *
     * @param _eloop The EventLoop.
     * @param _configs Use these config files.
     * @param _cmd Read from this command line.
     * @param ft_node_type The type of node that this runtime will be used for.
     * @param _slate_syncer_duplex_mode The duplex mode SlateSyncer should
     *                                  operate with.
     * @param _slate_syncer_synchronous_send True to send the sync shard as soon
     *                                       as it is prepared, instead of
     *                                       during slack time. See the header
     *                                       file for more information.
     * @param _clear_input_channels Set to true if data in all input channels
     *                             should be cleared at the end of the control
     *                             cycle.
     * @param _compute_data_sharing_input_crc True to compute the data sharing
     *                                        input_crc each cycle. False to
     *                                        skip  this computation and leave
     *                                        the input_crc at zero.
     * @param _use_sensor_prefixes Use sensor prefixes from parsed configs.
     * @param _enable_local_ext_cmd True to set up the local (non-synchronized)
     *        external commanding system.
     * @param _ext_cmd_params The set of parameters used to configure the local
     *                        (non-synchronized) ExternalCommandDispatcher.
     *                        Only used if local commanding is enabled by
     *                        `_enable_local_ext_cmd`.
     * @param _control A handle to the instance of the control logic that this
     *                 runtime class is wrapping. Each subclass of
     *                 FtRuntime should instantiate and pass in the
     *                 corresponding subclass of ControlInterface.
     */
    FtRuntime::FtRuntime(
        EventLoop &_eloop, const Configs &_configs, const CmdLine &_cmd,
        const ft_node_type_t ft_node_type,
        const slate_syncer_duplex_mode_t _slate_syncer_duplex_mode,
        const bool _slate_syncer_synchronous_send,
        const bool _clear_input_channels,
        const bool _compute_data_sharing_input_crc,
        const bool _use_sensor_prefixes, bool _enable_local_ext_cmd,
        const ExternalCommandDispatcher::config_params_t &_ext_cmd_params,
        Handle<ControlInterface> _control)
        : props(ft_node_type),
          slate_syncer_duplex_mode(_slate_syncer_duplex_mode),
          slate_syncer_synchronous_send(_slate_syncer_synchronous_send),
          clear_input_channels(_clear_input_channels),
          compute_data_sharing_input_crc(_compute_data_sharing_input_crc),
          use_sensor_prefixes(_use_sensor_prefixes),
          enable_local_external_commanding(_enable_local_ext_cmd),
          ext_cmd_params(_ext_cmd_params), eloop(_eloop),
          upkeep_list(eloop, "upkeep_list"), configs(_configs), cmd(_cmd),
          smoketest_config(SmoketestConfig::create(_cmd)), control(_control),
          is_init(false), control_period(0), ds_parallel_sleep_time(0), ident(),
          node_configs(), slate_root(), slate_local(),
          slate_control_read_create(), slate_control_read_only(), slate(),
          slate_syncer_replace_timer(eloop.clock),
          firmware_comm_timer(eloop.clock), pre_data_sharing_timer(eloop.clock),
          data_sharing_timer(eloop.clock), post_data_sharing_timer(eloop.clock),
          pre_telemetry_timer(eloop.clock), telemetry_timer(eloop.clock),
          slate_syncer_share_timer(eloop.clock),
          slate_roll_frame_timer(eloop.clock),
          channel_manager_flush_timer(eloop.clock),
          eloop_dispatch_select_timer(eloop.clock), bootstrapper(), ft_sync(),
          keychain(), channel_manager(), node_io_manager(), data_sharer(),
          reset_counters_trap(), adc_boards_info(), firmware_comm(),
          adc_scaler_local(), slate_sender_local(), slate_syncer(),
          slate_syncer_fd_bag(), slate_command_interface_local(),
          reflection_manager_local(), gnd_cmd_dispatcher_nonsynced(),
          enum_registry(new EnumRegistry(EnumRegistry::active)),
          scale_info(new scale_info_v), scaled_only_telemetry_channels(),
          telem_relay(), timestamp_gatherer(), slate_telem(), null_int_tok(),
          null_int_address_tok(), simulated_control_cycle_extension_tok(),
          enable_simulated_control_cycle_extension_tok(), heap_used_tok(),
          heap_max_used_tok(), heap_allocations_count_tok(),
          display_heap_allocation_locations_periodic(
              "display_allocation_locations", 300 * billion),
          ftrace_trap(), ftrace_trap_local(), dna(), slate_dump(), time_scaler()
    {
        /*
         * Ensure that the child class has passed us a valid
         * ControlInterface object.
         */
        SacAssert(control);
    }
    /**
     * Destructor.
     */
    FtRuntime::~FtRuntime() {}
    /**
     * Initialize FtRuntime. This creates and registers all members.
     *
     * @return True on success.
     */
    bool FtRuntime::init()
    {
        SacAbortIf(is_init, false);
        /*
         * Enable the configuration cache to reduce boot time.
         */
        enable_config_cache = false;
        /*
         * Initialize any runtime systems that must exist prior to starting
         * initialization of the control component.
         */
        SacAbortIfNot(init_pre_control_early(), false);
        /*
         * Perform pre-slate-build initialization of the control logic.
         */
        ControlNodeIdentity control_ident;
        SacAbortIfNot(control_ident.init(ident.role, get_control_inst()),
                      false);
        /*
         * Read the node configs from the Node Manager config file.
         */
        const std::string node_mgr_config_key = "node_mgr";
        SacAbortIfNot(node_configs.parse(configs, node_mgr_config_key), false);
        SacAbortIf(node_configs.empty(), false);
        /*
         * Provide the control component with fundamental information about its
         * identity, and allow it to append requests for additional remote I/O
         * nodes to node_configs.
         */
        SacAbortIfNot(control->init_basic_identity(
                          control_period, configs,
                          smoketest_config.get_prog_name(), control_ident,
                          slate_control_sync_only, node_configs),
                      false);
        SacAbortIfNot(init_pre_control_late(), false);
        SacAbortIfNot(init_runtime_pre_control(), false);
        cycle_timer_name_s control_cycle_timer_names;
        SacAbortIfNot(control->init_pre_slate_build(
                          keychain, channel_manager, adc_boards_info,
                          enum_registry, control_cycle_timer_names),
                      false);
        /*
         * Perform pre-slate-build initialization of runtime systems that may
         * rely on the existence of control components.
         */
        SacAbortIfNot(init_pre_slate_build_early(), false);
        SacAbortIfNot(init_runtime_pre_slate_build(), false);
        SacAbortIfNot(init_pre_slate_build_late(), false);
        /*
         * Create all the requested CycleTimers.
         */
        cycle_timer_m control_cycle_timers;
        for (cycle_timer_name_s::const_iterator i =
                 control_cycle_timer_names.begin();
             i != control_cycle_timer_names.end(); ++i)
        {
            Handle<CycleTimer> timer(new CycleTimer(eloop.clock));
            SacAbortIfNot(timer->init(slate_local, *i), false);
            control_cycle_timers[*i] = timer;
        }
        /*
         * Initialize the time scaler.
         */
        SacAbortIfNot(time_scaler.init(slate_root.super_slate()), false);
        /*
         * Build the slate.
         */
        slate = slate_root.slate(slate_no_validation);
        SacAbortIfNot(slate_root.build(), false);
        /*
         * Perform post-slate build initialization of the control code.
         */
        SlateBuilder sudo_slate_root = slate_root.super_slate();
        SlateBuilder sudo_slate_control =
            sudo_slate_root.sub_slate(control_slate_name());
        SacAbortIfNot(control->init_post_slate_build(sudo_slate_control,
                                                     control_cycle_timers),
                      false);
        /*
         * Perform post-slate-build initialization of all runtime components.
         */
        SacAbortIfNot(init_post_slate_build_early(), false);
        SacAbortIfNot(init_runtime_post_slate_build(), false);
        SacAbortIfNot(init_post_slate_build_late(), false);
        /*
         * Validate that all seed objects have been initialized.
         */
        SacAbortIfNot(validate_all_seeds_initialized(), false);
        /*
         * Log all initial values to DNA.
         */
        SacIfNot(dna.log(0, eloop.clock.get_telemetry_timestamp()));
        /*
         * Ensure NOTHING was added to any of the EventLoop lists before
         * we got to this point, and install the dispatch method.
         */
        SacAbortIfNeq(eloop.early_list.num_sources(), 0U, false);
        SacAbortIfNeq(eloop.normal_list.num_sources(), 0U, false);
        SacAbortIfNeq(eloop.late_list.num_sources(), 0U, false);
        SacAbortIfNot(install_dispatch(eloop, "dispatch",
                                       make_slot(*this, &FtRuntime::dispatch)),
                      false);
        /*
         * Disable and clear the config cache once initialization is complete.
         */
        enable_config_cache = false;
        clear_config_cache();
        /*
         * Finalize slate, clearing any build time only metadata.
         */
        SacAbortIfNot(slate_root.finalize(), false);
        is_init = true;
        return true;
    }
    /**
     * Print all the slate elements to stdout.
     *
     * @return True on success.
     */
    bool FtRuntime::dump_devices()
    {
        str_v devices;
        slateelem_v elements;
        SacAbortIfNot(slate_root.super_slate().compute_path_set(elements),
                      false);
        for (size_t i = 0; i < elements.size(); ++i)
        {
            devices.push_back(elements[i].path);
        }
        /*
         * Sort the list.
         */
        std::sort(devices.begin(), devices.end());
        const size_t devices_size = devices.size();
        fprintf(stdout, "[");
        for (size_t i = 0; i < devices_size; ++i)
        {
            if (i)
            {
                fprintf(stdout, ",");
            }
            fprintf(stdout, "[\"%s\"]", devices[i].c_str());
        }
        fprintf(stdout, "]");
        fflush(stdout);
        return true;
    }
    /**
     * Dispatch all vehicle control components.
     *
     * @return nano_t_min always to ensure that the EventLoop calls us back
     *         immediately. This allows FtSync to govern control cycle timing.
     */
    nano_t FtRuntime::dispatch(nano_t)
    {
        /*
         * Dispatch non-synced components that should be executed prior to the
         * synchronized portion of the control cycle. This does not need to be
         * hotsyncable.
         */
        dispatch_nonsynced_early();
        /*
         * If synchronization has been achieved, dispatch all synchronized
         * components. These components should behave identically on all
         * strings, and therefore should be hotsyncable.
         */
        if (bootstrapper->is_synced())
        {
            /*
             * (HOOTL) Hint to chronos that we're doing computations and it can
             * parallelize other processes.
             */
            time_slave_control_begin();
            /*
             * Dispatch synchronized code.
             */
            control->dispatch_synced();
            /*
             * (HOOTL) Hint to chronos that we are done with our computations.
             */
            time_slave_control_end();
            /*
             * Update the clock telemetry timestamp. This must be done after
             * the synchronized code ran and only if it ran.
             */
            timestamp_gatherer->update_clock_telemetry_timestamp();
        }
        /*
         * Dispatch non-synced components that should be executed subsequent to
         * the synchronized portion of the control cycle. This does not need to
         * be hotsyncable.
         */
        dispatch_nonsynced_late();
        /*
         * Return nano_t_min so the EventLoop calls us back immediately.
         */
        return nano_t_min;
    }
    /**
     * Initialize any runtime systems that must exist prior to providing the
     * control component with the most basic information about its identity.
     *
     * @return True on success.
     */
    bool FtRuntime::init_pre_control_early()
    {
        SacAbortIf(is_init, false);
        /*
         * Store off all of the commonly-used names for future use.
         */
        SacAbortIfNot(ident.init(cmd.get_string("role"), cmd.get_int("inst"),
                                 cmd.get_string("string")),
                      false);
        /*
         * Check if the DNA debugger is present and potentially connect to it.
         * This needs to be done first since this returns the slate root to use.
         */
        SacAbortIfNot(
            dna.init(ident.node_name, local_slate_name() + ".dna", &slate_root),
            false);
        /*
         * The telemetry root is slate_root, and enum_registry must be added
         * before sub-slates are created.
         */
        slate_root.add_enum_registry(enum_registry);
        /*
         * Create our slates. Non-control slates cannot create elements in the
         * sync shard.
         */
        const slate_permission_t runtime_permission = slate_permission_deny(
            slate_permission_rwc, slate_permission_c_sync);
        slate_local =
            slate_root.sub_slate(local_slate_name(), runtime_permission);
        slate_control_read_only =
            slate_root.sub_slate(control_slate_name(), slate_permission_r);
        /*
         * Temporarily create a control slate that allows creation and reading
         * of elements. This will eventually need to be removed, tracked by
         * #14879.
         */
        slate_control_read_create =
            slate_root.sub_slate(control_slate_name(), slate_permission_rc);
        /*
         * Create a control slate that allows reading, writing, and creation of
         * synced elements, to be passed to the control instance.
         */
        const slate_permission_t control_permission = slate_permission_deny(
            slate_permission_rwc, slate_permission_c_nonsync);
        slate_control_sync_only =
            slate_root.sub_slate(control_slate_name(), control_permission);
        /*
         * Create and initialize FtSync and use it to obtain the control
         * period.
         */
        ft_sync = Handle<FtSync>(
            new FtSync(eloop, upkeep_list, false /* auto_send_syncs */));
        SacAbortIfNot(ft_sync, false);
        SacAbortIfNot(ft_sync->init(slate_local, configs, cmd), false);
        SacAbortIfNot(
            ft_sync->populate_enums(*enum_registry, local_enum_prefix()),
            false);
        control_period = ft_sync->get_sync_period();
        return true;
    }
    /**
     * Initialize any runtime systems that must exist prior to calling
     * ControlInterface::init_pre_slate_build().
     *
     * @return True on success.
     */
    bool FtRuntime::init_pre_control_late()
    {
        if (cmd.get_bool("enable_slate_dump"))
        {
            slate_dump.assume_ownership(new SlateDump(eloop.clock));
            SacAbortIfNot(slate_dump->init(slate_local, smoketest_config),
                          false);
        }
        /*
         * Create the Keychain and load our private key.
         */
        keychain = Handle<Keychain>(new Keychain(Keychain::use_murmurhash));
        SacAbortIfNot(keychain, false);
        SacAbortIfNot(keychain->init(configs, ident.node_name, "public.key"),
                      false);
        SacAbortIfNot(
            keychain->load_private_key(configs, ident.node_name + ".key"),
            false);
        /*
         * Create heap usage tokens.
         */
        SacAbortIfNot(slate_local.create("heap.bytes", shard_nonsync,
                                         slate_private, heap_used_tok),
                      false);
        SacAbortIfNot(slate_local.create("heap.max_bytes", shard_nonsync,
                                         slate_private, heap_max_used_tok),
                      false);
        SacAbortIfNot(slate_local.create("heap.allocations", shard_nonsync,
                                         slate_private,
                                         heap_allocations_count_tok),
                      false);
        /*
         * Create control cycle timers.
         */
        SlateBuilder slate_prof = slate_local.sub_slate("prof");
        SacAbortIfNot(
            slate_syncer_replace_timer.init(slate_prof, "slate_syncer_replace"),
            false);
        SacAbortIfNot(firmware_comm_timer.init(slate_prof, "firmware_comm"),
                      false);
        SacAbortIfNot(
            pre_data_sharing_timer.init(slate_prof, "pre_data_sharing"), false);
        SacAbortIfNot(data_sharing_timer.init(slate_prof, "data_sharing"),
                      false);
        SacAbortIfNot(
            post_data_sharing_timer.init(slate_prof, "post_data_sharing"),
            false);
        SacAbortIfNot(pre_telemetry_timer.init(slate_prof, "pre_telemetry"),
                      false);
        SacAbortIfNot(telemetry_timer.init(slate_prof, "telemetry"), false);
        SacAbortIfNot(
            slate_syncer_share_timer.init(slate_prof, "slate_syncer_share"),
            false);
        SacAbortIfNot(
            slate_roll_frame_timer.init(slate_prof, "slate_roll_frame"), false);
        SacAbortIfNot(channel_manager_flush_timer.init(slate_prof,
                                                       "channel_manager_flush"),
                      false);
        SacAbortIfNot(eloop_dispatch_select_timer.init(slate_prof,
                                                       "eloop_dispatch_select"),
                      false);
        /*
         * Create the ADC system which will communicate with the
         * hardware.
         */
        if (props.use_firmware_comm)
        {
            SacAbortIfNot(create_adc_system(), false);
        }
        /*
         * Create the channel manager so we can bind to FT channels.
         */
        SacAbortIf(node_configs.empty(), false);
        SacAbortIfNot(channel_manager.assume_ownership(
                          new FtChannelManager(ident.string)),
                      false);
        SacAbortIfNot(channel_manager->init(node_configs), false);
        /*
         * Create the input sharing system, which will exchange and vote
         * input data between computers.
         */
        nano_t share_time = 0;
        SacAbortIfNot(
            string_to_nano_t(cmd.get_string("ds_share_time"), share_time),
            false);
        nano_t reshare_time = 0;
        SacAbortIfNot(
            string_to_nano_t(cmd.get_string("ds_reshare_time"), reshare_time),
            false);
        if (props.use_input_sync)
        {
            SacAbortIfNot(create_input_sharing_system(share_time, reshare_time),
                          false);
        }
        else
        {
            /*
             * On systems that do not use input sync, we instead sleep for the
             * duration of share time and reshare time to keep them in phase
             * with their counterparts, which will be selecting for this amount
             * of time in the data sharing algorithm. See #10938 and #11122 for
             * more details.
             */
            ds_parallel_sleep_time = share_time + reshare_time;
            /*
             * Nodes without input sync also need some devices that the data
             * sharer would have created. Create them here.
             */
            slate_element_t element_id = 0;
            SacAbortIfNot(slate_local.create_element<bool>(
                              "ds.trigger", false, shard_nonsync,
                              slate_read_write, element_id),
                          false);
            SacAbortIfNot(slate_local.create_element<UINT32>(
                              "ds.input_crc", 0u, shard_nonsync,
                              slate_read_write, element_id),
                          false);
        }
        /*
         * Create the state sync system.
         */
        if (props.use_slate_syncer)
        {
            SacAbortIfNot(slate_syncer_fd_bag.assume_ownership(new FdBag),
                          false);
            /*
             * Put SlateSyncer file descriptors in their own FdBag so they can
             * be dispatched separately for transmitting, if needed. Register
             * this bag with FtSync so it gets dispatched during slack time.
             *
             * If slate_syncer_synchronous_send is enabled, this FdBag will be
             * dispatched for transmission before FtRuntime yields back to the
             * event loop, and receipt will occur during slack time.
             *
             * Otherwise, both transmission and receipt will occur during slack
             * time.
             */
            SacAbortIfNot(slate_syncer.assume_ownership(new SlateSyncer(
                              eloop.clock, *slate_syncer_fd_bag,
                              slate_syncer_duplex_mode, control_period)),
                          false);
            SacAbortIfNot(ft_sync->add_extra_fd_bag(slate_syncer_fd_bag),
                          false);
            const bool start_enabled = false;
            SacAbortIfNot(
                slate_syncer->init_outputs(slate_local, ident.role_inst,
                                           ident.string, start_enabled),
                false);
        }
        return true;
    }
    /**
     * Initialize common components that rely on the existence of control
     * systems, but are in turn relied upon by vehicle-specific runtime
     * components.
     *
     * @return True on success.
     */
    bool FtRuntime::init_pre_slate_build_early()
    {
        SacAbortIf(is_init, false);
        /*
         * Finish creation of the slate syncer system.
         */
        if (props.use_slate_syncer)
        {
            SlateBuilder slate_shared_a =
                slate_control_read_only.sub_slate(shared_slate_name('a'));
            SlateBuilder slate_shared_b =
                slate_control_read_only.sub_slate(shared_slate_name('b'));
            SlateBuilder slate_shared_c =
                slate_control_read_only.sub_slate(shared_slate_name('c'));
            SlateBuilder slate_shared_median =
                slate_control_read_only.sub_slate(median_slate_name());
            SacAbortIfNot(slate_syncer->init_inputs(
                              slate_shared_a, slate_shared_b, slate_shared_c,
                              slate_shared_median, ident.string),
                          false);
        }
        /*
         * Create the trap that signals the world about counter reset commands.
         * Note that the trap binds to the reset_counters element in the
         * control slate that is actuated by the state machine.
         */
        SacAbortIfNot(reset_counters_trap.assume_ownership(new SlateFlagTrap),
                      false);
        SacAbortIfNot(reset_counters_trap->init(slate_control_read_only,
                                                "reset_counters", slate_local,
                                                "reset_counters_trap_local",
                                                shard_nonsync),
                      false);
        SacAbortIfNot(reset_counters_trap->trap_sig.connect(
                          make_slot(*this, &FtRuntime::reset_counters)),
                      false);
        /*
         * Create and initialize the TimestampGatherer.
         */
        SacAbortIfNot(timestamp_gatherer.assume_ownership(
                          new TimestampGatherer(eloop.clock)),
                      false);
        SacAbortIfNot(
            timestamp_gatherer->init(slate_local, slate_control_read_only),
            false);
        /*
         * Create the telemetry relay, but don't finalize it yet.
         */
        SacAbortIfNot(telem_relay.assume_ownership(
                          new TelemetryRelayRuntime(eloop.clock, "telem")),
                      false);
        SacAbortIfNot(telem_relay, false);
        SacAbortIfNot(create_local_telemetry_connections(*telem_relay), false);
        SacAbortIfNot(
            telem_relay->init(eloop, upkeep_list, slate_control_read_only,
                              slate_local, configs, ident,
                              get_telemetry_config_file_name(),
                              get_muxed_telemetry_groups(),
                              should_telem_relay_aggregate_dest_connections()),
            false);
        /*
         * Obtain a read token to the null_int device so we can telemeter its
         * address for use in desync testing.
         */
        SacAbortIfNot(slate_control_read_only.bind("null_int", null_int_tok),
                      false);
        SacAbortIfNot(slate_local.create("null_int_address", 0llu,
                                         shard_nonsync, slate_private,
                                         null_int_address_tok),
                      false);
        /*
         * Create and initialize the devices that allow the control cycle to be
         * artifically extended.
         *
         * WARNING: This mechanism is meant to be used during testing ONLY. It
         *          is imperative that the extension be initialized to 0.0 and
         *          the enable device be initialized to false to prevent
         *          accidental activation in flight.
         */
        SacAbortIfNot(slate_local.create("simulated_control_cycle_extension",
                                         0.0, shard_nonsync, slate_private,
                                         simulated_control_cycle_extension_tok),
                      false);
        SacAbortIfNot(
            slate_local.create("enable_simulated_control_cycle_extension",
                               false, shard_nonsync, slate_private,
                               enable_simulated_control_cycle_extension_tok),
            false);
        /*
         * Initialize the ftrace library and ftrace trap, if so configured.
         * ftrace_init(false) will return true even if tracing is not
         * enabled by the system. It does not guarantee initialization.
         */
        if (props.use_ftrace_trap && !cmd.get_bool("disable_ftrace"))
        {
            SacAbortIfNot(ftrace_init(false /* report_error */), false);
            ftrace_trap = FtraceTrap::create(
                eloop.clock, slate_control_read_only, slate_local);
            SacAbortIfNot(ftrace_trap, false);
            ftrace_trap_local =
                FtraceTrap::create(eloop.clock, slate_control_read_only,
                                   slate_local, ident.node_name);
            SacAbortIfNot(ftrace_trap_local, false);
        }
        /*
         * Create the non-synced command platform.
         */
        if (enable_local_external_commanding)
        {
            SacAbortIfNot(create_nonsynced_command_platform(), false);
        }
        return true;
    }
    /**
     * Initialize common components that rely on the existence of control
     * systems and vehicle-specific runtime components.
     *
     * @return True on success.
     */
    bool FtRuntime::init_pre_slate_build_late()
    {
        SacAbortIf(is_init, false);
        /*
         * Create and initialize the bootstrapper system. Happens before local
         * slate sharing system creation so that we can share the bootstrapper
         * state.
         */
        SacAbortIfNot(create_bootstrapper_system(), false);
        /*
         * Create and initialize the local slate sharing system.
         */
        if (props.use_input_sync)
        {
            /*
             * Our own local slate sharing message only needs to go into data
             * sharing, so we'll set up a channel that we'll listen on and then
             * make it look like the data came from our node via the node IO
             * manager.
             */
            size_t buf_size = 0;
            SacAbortIfNot(node_configs.get_node_output_buffer_size(
                              ident.node_name, buf_size),
                          false);
            Handle<DataDgramChannel> channel(new DataDgramChannel(1, buf_size));
            SacAbortIfNot(channel, false);
            node_id_t node_id = unknown_node_id;
            SacAbortIfNot(
                NodeIdentifier::node_name_to_node_id(ident.node_name, node_id),
                false);
            SacAbortIfNot(
                channel->read_sig.connect(slot_bind(
                    make_slot(*this, &FtRuntime::handle_local_share_read),
                    node_id)),
                false);
            /*
             * The local slate share sender is rooted at the local slate. There
             * is no need to share anything from the control slate, as all
             * strings will have identical values.
             */
            SacAbortIf(slate_sender_local, false);
            SacAbortIfNot(slate_sender_local.assume_ownership(
                              new SlateSharerSender(eloop.clock)),
                          false);
            SacAbortIfNot(
                slate_sender_local->init("local_share_sender", slate_local,
                                         shard_nonsync, configs,
                                         sharer_get_default_local_config_list(
                                             props.use_firmware_comm),
                                         channel, sharer_bind),
                false);
        }
        /*
         * Block the further addition of enumerations to the enum registry.
         */
        SacAbortIfNot(enum_registry->finalize(slate_root.super_slate()), false);
        if (props.use_input_sync)
        {
            /*
             * Some nodes call init_output_sources at different times. Ensure
             * that by this point NodeIOManager is fully initialized on all
             * nodes.
             */
            SacAbortIfNot(node_io_manager->is_initialized(), false);
        }
        return true;
    }
    /**
     * Perform post-slate-build initialization of common components that
     * vehicle-specific runtime components rely on for finalization.
     *
     * @return True on success.
     */
    bool FtRuntime::init_post_slate_build_early()
    {
        SacAbortIf(is_init, false);
        /*
         * Print shard statistics to the console.
         */
        std::pair<std::string, slate_shard_t> shard_desc[num_slate_shard_t];
        shard_desc[shard_static] = std::make_pair("static", shard_static);
        shard_desc[shard_sync] = std::make_pair("sync", shard_sync);
        shard_desc[shard_nonsync] = std::make_pair("nonsync", shard_nonsync);
        shard_desc[shard_cyclic] = std::make_pair("cyclic", shard_cyclic);
        shard_desc[shard_sync_no_telem] =
            std::make_pair("sync (non-telemetered)", shard_sync_no_telem);
        shard_desc[shard_nonsync_no_telem] =
            std::make_pair("nonsync (non-telemetered)", shard_nonsync_no_telem);
        shard_desc[shard_cyclic_no_telem] =
            std::make_pair("cyclic (non-telemetered)", shard_cyclic_no_telem);
        static_assert(DIM(shard_desc) == 7);
        for (size_t i = 0; i < num_slate_shard_t; i++)
        {
            const std::string &shard_name = shard_desc[i].first;
            const slate_shard_t shard = shard_desc[i].second;
            B2c mem;
            SacAbortIfNot(slate.get_shard_memory(shard, mem), false);
            dbnprintf(200, "Slate '%s' shard size: %zukb\n", shard_name.c_str(),
                      mem.len() / 1024);
        }
        /*
         * Finalize the state syncer system.
         */
        if (props.use_slate_syncer)
        {
            SacAbortIfNot(slate_syncer, false);
            SacAbortIfNot(slate_syncer->init(), false);
        }
        /*
         * Set up the local and control sudo slates.
         */
        SlateBuilder sudo_slate_root = slate_root.super_slate();
        SlateBuilder sudo_slate_local =
            sudo_slate_root.sub_slate(local_slate_name());
        SlateBuilder sudo_slate_control =
            sudo_slate_root.sub_slate(control_slate_name());
        /*
         * Finalize FirmwareComm.
         */
        if (props.use_firmware_comm)
        {
            const bool sim_firmware = cmd.get_bool("sim_firmware");
            SacAbortIfNot(firmware_comm->init_controllers(sim_firmware,
                                                          false /* resume */),
                          false);
        }
        /*
         * Initialize the Slate command interface.
         */
        if (slate_command_interface_local)
        {
            SacAbortIfNot(slate_command_interface_local, false);
            SacAbortIfNot(slate_command_interface_local->init(sudo_slate_local,
                                                              ident.node_name),
                          false);
        }
        /*
         * Dump ADC info to telemetry scaling.
         */
        std::set<std::string> hardware_prefixes;
        /*
         * Local slate and locally-shared hardware inputs appear in
         * telemetry at the same level due to double mounting.
         */
        hardware_prefixes.insert(shared_slate_name('a') + ".");
        hardware_prefixes.insert(shared_slate_name('b') + ".");
        hardware_prefixes.insert(shared_slate_name('c') + ".");
        /*
         * Medianed inputs and hardware outputs.
         */
        hardware_prefixes.insert(control_slate_name() + ".");
        SacAbortIfNot(adc_boards_to_scale_info(hardware_prefixes,
                                               adc_boards_info, *scale_info),
                      false);
        /*
         * We normally telemeter the ADC counts from the raw subslate of the
         * local or control slates, and use telemetry annotations for scaling.
         * We can't do that for updatable calibrations since the telemetry
         * annotations don't update, so erase scale_info for those elements.
         * This forces those channels to be telemetered as floats, which are
         * correctly scaled by the SensorCals class.
         */
        scale_info->erase(
            std::remove_if(scale_info->begin(), scale_info->end(),
                           [this](const scale_info_t &info) {
                               return scaled_only_telemetry_channels.count(
                                          info.name) > 0;
                           }),
            scale_info->end());
        /*
         * Add any scaling info generated by control logic into our list.
         */
        scale_info_v control_scaling;
        SacAbortIfNot(control->get_scaling(control_scaling), false);
        scale_info->insert(scale_info->end(), control_scaling.begin(),
                           control_scaling.end());
        sort_scale_info(*scale_info);
        return true;
    }
    /**
     * Perform post-slate-build initialization of common components that rely
     * on vehicle-specific runtime components being finalized.
     *
     * @return True on success.
     */
    bool FtRuntime::init_post_slate_build_late()
    {
        SacAbortIf(is_init, false);
        if (slate_dump)
        {
            SacAbortIfNot(slate_dump->init_post_slate(
                              ident, control_period, *enum_registry,
                              smoketest_config.get_init_only()),
                          false);
        }
        /*
         * Allow the control slate to be referenced in telemetry with no
         * prefix.
         */
        const std::string device_root = control_slate_name();
        RemoteDeviceConfig::str_parse_fn_m telemetry_parsers;
        /*
         * Make the SlateTelemetryTask.
         */
        SacAbortIfNot(slate_telem.assume_ownership(new SlateTelemetryTask()),
                      false);
        SacAbortIfNot(slate_telem->init(
                          slate_root.super_slate(), eloop.clock, *telem_relay,
                          ident, *scale_info, *enum_registry, telemetry_parsers,
                          smoketest_config, device_root, control_period),
                      false);
        /*
         * Add the SlateTelemetryTask telemetry producer.
         */
        SacAbortIfNot(telem_relay->add_producer(slate_telem), false);
        /*
         * Write annotations for the devices and slate telemetry tasks.
         */
        SacAbortIfNot(telem_relay->finalize(smoketest_config), false);
        return true;
    }
    /**
     * Vehicles should use this method to initialize any systems that must
     * exist prior to initializing the control components.
     *
     * @return True on success.
     */
    bool FtRuntime::init_runtime_pre_control()
    {
        SacAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to perform pre-slate-build
     * initialization of systems that may rely on the existence of control
     * components.
     *
     * @return True on success.
     */
    bool FtRuntime::init_runtime_pre_slate_build()
    {
        SacAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to initialize systems that require slate
     * to have already been built.
     *
     * @return True on success.
     */
    bool FtRuntime::init_runtime_post_slate_build()
    {
        SacAbortIf(is_init, false);
        return true;
    }
    /**
     * Dispatch components that run prior to the synchronized portion of the
     * control cycle.
     */
    void FtRuntime::dispatch_nonsynced_early()
    {
        /*
         * Record the time that the EventLoop spent dispatching select events
         * last cycle. Do this before running FtSync and starting the next
         * cycle.
         *
         * EventLoop::dispatch_events() dispatches event lists before select
         * events. That means that on the first run through the event loop at
         * program start, this method gets invoked before select events have
         * ever been dispatched, and EventLoop's select event start/stop times
         * are zero. To put it another way, on the first control cycle, there is
         * no previous cycle's select events to report timing for.
         *
         * CycleTimer computes some timing metrics relative to the start of the
         * control cycle, so it requires that the passed start and stop times
         * fall after the start of the current cycle. This means we can't pass
         * through the zeros from EventLoop's timers on the first cycle --
         * instead, just leave the CycleTimer alone so it reports zero time for
         * the previous (non-existent) cycle.
         */
        if (eloop.dispatch_select_start_time != 0)
        {
            eloop_dispatch_select_timer.start(eloop.dispatch_select_start_time);
            eloop_dispatch_select_timer.stop(eloop.dispatch_select_stop_time);
        }
        /*
         * Run FtSync first. This is necessary because it can set the
         * EventLoop's control time, so all other components MUST come
         * afterwards to ensure that they use a consistent control time.
         *
         * FtSync doesn't read control time.
         */
        ft_sync->dispatch(0);
        /*
         * Let the control know about the beginning of the new cycle.
         */
        control->start_cycle(eloop.clock.control_time());
        /*
         * Hotsync our state from another peer if we have been rebooted or have
         * desynced from our peers. Note that although this function will be
         * executed on the flight computers, it will never do anything because
         * they have hotsync disabled.
         */
        slate_syncer_replace_timer.start();
        if (props.use_slate_syncer)
        {
            slate_syncer->dispatch_process(eloop.control_time());
        }
        slate_syncer_replace_timer.stop();
        firmware_comm_timer.start();
        /*
         * Dispatch any non-synced components that must run prior to writing
         * outputs but after unscaling has taken place (e.g., components that
         * write to the raw slate directly).
         */
        dispatch_nonsynced_pre_firmware_comm();
        if (props.use_firmware_comm)
        {
            /*
             * Copy output data from this process to the DMA driver.
             */
            firmware_comm->write_outputs();
            /*
             * Copy input data from the DMA driver to this process. This
             * input data was read from hardware via a DMA transfer that
             * was scheduled during the last cycle to occur just before
             * the beginning of this cycle. This also reads the firmware
             * ready flag, which is checked in dispatch_outputs().
             */
            firmware_comm->read_inputs();
            /*
             * Commit the values currently in the DMA driver to hardware.
             */
            firmware_comm->dispatch_outputs(eloop.control_time());
            /*
             * Schedule a DMA read for the next cycle - this call cannot be
             * moved above read_inputs(), because there are immediate side
             * effects of calling this that would adversely affect
             * read_inputs().
             */
            firmware_comm->update_inputs();
            /*
             * Scale raw data into the local slate. This is done here instead of
             * in dispatch_synced to enable DIO telemetry before sync.
             *
             * ADC channels are telemetered directly from the raw slate.
             */
            adc_scaler_local->read_scaled();
        }
        firmware_comm_timer.stop();
        pre_data_sharing_timer.start();
        /*
         * Store local telemetry timestamp into slate. This must be done before
         * local slate sharing so that the most recent value gets shared.
         */
        timestamp_gatherer->update_local_timestamp();
        /*
         * Dispatch non-synced commands.
         */
        if (gnd_cmd_dispatcher_nonsynced)
        {
            gnd_cmd_dispatcher_nonsynced->dispatch();
        }
        /*
         * Dispatch anything else which needs to be run before data sharing.
         */
        dispatch_nonsynced_pre_data_sharing();
        if (props.use_input_sync)
        {
            /*
             * Send sensor data to peers.
             */
            slate_sender_local->share();
        }
        pre_data_sharing_timer.stop();
        data_sharing_timer.start();
        if (props.use_input_sync)
        {
            /*
             * The data sharer should be run before any control processing is
             * done to ensure that all input data is read and shared before the
             * rest of the system tries to use it.
             */
            data_sharer->dispatch(eloop.control_time());
        }
        else
        {
            /*
             *  Use the ds_dispatch_begin access call to inform Chronos
             *  that synchronization has been achieved.
             */
            time_slave_ds_dispatch_begin(bootstrapper->is_synced());
            /*
             * On systems that do not participate in input synchronization, we
             * sleep for an equivalent period of time.
             */
            sxsleep(ds_parallel_sleep_time);
        }
        data_sharing_timer.stop();
        post_data_sharing_timer.start();
        /*
         * Dispatch anything which needs to be run immediately after data
         * sharing.
         */
        dispatch_nonsynced_post_data_sharing();
        if (props.use_slate_syncer)
        {
            /*
             * Check state sync before any control code has a chance to run.
             */
            slate_syncer->dispatch_check(eloop.control_time());
        }
        /*
         * Dispatch the FtBootstrapper. This is done before dispatch_synced so
         * we can detect synchronization and dispatch the synchronized
         * components on the same cycle.
         */
        bootstrapper->dispatch();
        /*
         * Send out time-sync messages.
         */
        if (!should_send_syncs_during_reshare_phase())
        {
            ft_sync->send_syncs();
        }
        post_data_sharing_timer.stop();
    }
    /**
     * Dispatch components that run subsequent to the synchronized portion of
     * the control cycle.
     */
    void FtRuntime::dispatch_nonsynced_late()
    {
        const nano_t pre_telemetry_timer_start_time =
            Clock::get_monotonic_time();
        /*
         * Artificially delay the remainder of the control cycle by sleeping
         * for the specified period, if so enabled.
         *
         * WARNING: This mechanism is meant to be used during testing ONLY.
         */
        if (slate[enable_simulated_control_cycle_extension_tok])
        {
            const nano_t control_cycle_extension = static_cast<nano_t>(
                slate[simulated_control_cycle_extension_tok] * billion);
            sxsleep(control_cycle_extension);
        }
        /*
         * Dispatch the reset_counter_trap so the counter-clearing methods can
         * take effect before telemetry is generated. This trap watches a
         * device commanded by the state machine, so it must run after the
         * control code.
         */
        reset_counters_trap->dispatch();
        /*
         * Do generic upkeeping.
         */
        upkeep_list.dispatch(eloop.control_time());
        /*
         * Update heap allocation statistics.
         */
        slate[heap_used_tok] = get_heap_allocation();
        slate[heap_max_used_tok] = get_max_heap_allocation();
        slate[heap_allocations_count_tok] = get_heap_allocations_count();
        if (display_heap_allocation_locations_periodic.is_due(
                eloop.control_time()))
        {
            display_heap_allocation_locations();
        }
        /*
         * Create an ftrace snapshot if one of the tracing alarm conditions
         * was tripped.
         */
        if (ftrace_trap)
        {
            ftrace_trap->dispatch();
        }
        if (ftrace_trap_local)
        {
            ftrace_trap_local->dispatch();
        }
        /*
         * Dispatch anything that needs to run as late as possible prior to
         * telemetry generation.
         */
        dispatch_nonsynced_pre_telemetry();
        /*
         * Telemeter the address of the null_int device for use in desync
         * testing.
         */
        const INT64 &null_int = slate[null_int_tok];
        slate[null_int_address_tok] = reinterpret_cast<UINT64>(&null_int);
        /*
         * Dispatch the device reflection managers just before the telemetry
         * relay so we get the most recent value of any reflected device.
         */
        if (reflection_manager_local)
        {
            reflection_manager_local->dispatch();
        }
        /**
         * Dispatch the time scaler.
         */
        time_scaler.dispatch();
        const nano_t pre_telemetry_timer_stop_time =
            Clock::get_monotonic_time();
        /*
         * Add telemetry to the end of the cycle.
         */
        const nano_t telemetry_timer_start_time = Clock::get_monotonic_time();
        telem_relay->dispatch(eloop.control_time());
        const nano_t telemetry_timer_stop_time = Clock::get_monotonic_time();
        /*
         * Log the content of this control cycle to the dna debugger.
         */
        dna.log(eloop.control_time(), eloop.clock.get_telemetry_timestamp());
        const nano_t slate_syncer_share_start_time =
            Clock::get_monotonic_time();
        if (props.use_slate_syncer)
        {
            /*
             * Compute the hash of our synchronized state, and prepare to share
             * it with our peers, in case they need to hotsync. This must happen
             * after all control code has run; no changes to the sync shard can
             * occur on this cycle after this point.
             */
            slate_syncer->dispatch_share(eloop.control_time());
            /*
             * If synchronous sending is enabled, try to send (but not receive)
             * data right now. This allows the (usually very large) sync shard
             * to flow while other outputs are emitted by the event loop.
             */
            if (slate_syncer_synchronous_send)
            {
                /*
                 * Perform a "select once", a select with zero timeout that
                 * will instantaneously poll the file descriptors, without
                 * blocking. Limit to fd_write_ev so only transmission occurs.
                 */
                nano_t time_used = nano_t_min;
                SacOnSelectError(slate_syncer_fd_bag->select_absolute(
                    nano_t_min, fd_write_ev, time_used));
            }
        }
        const nano_t slate_syncer_share_stop_time = Clock::get_monotonic_time();
        if (slate_dump)
        {
            slate_dump->set_synced(bootstrapper->is_synced());
            slate_dump->dispatch();
        }
        /*
         * Roll a new slate frame for the new cycle. It needs to be done at the
         * very end because anything with cyclic shard variables will have them
         * blown away after this point.
         *
         * SlateSyncer::dispatch_share() relies on SlateSharerReceiver outputs
         * which are in the cyclic shard on some vehicles, so it needs to be
         * dispatched before the frame is rolled.
         *
         * We also manually record start and stop CycleTimer times for
         * components which run after the telemetry has been dispatched but
         * before we roll the frame.
         */
        const nano_t slate_roll_start_time = slate_syncer_share_stop_time;
        slate.roll_frame();
        const nano_t slate_roll_stop_time = Clock::get_monotonic_time();
        pre_telemetry_timer.start(pre_telemetry_timer_start_time);
        pre_telemetry_timer.stop(pre_telemetry_timer_stop_time);
        telemetry_timer.start(telemetry_timer_start_time);
        telemetry_timer.stop(telemetry_timer_stop_time);
        slate_syncer_share_timer.start(slate_syncer_share_start_time);
        slate_syncer_share_timer.stop(slate_syncer_share_stop_time);
        slate_roll_frame_timer.start(slate_roll_start_time);
        slate_roll_frame_timer.stop(slate_roll_stop_time);
        /*
         * Flush any input that wasn't processed during this cycle.
         */
        channel_manager_flush_timer.start();
        if (clear_input_channels)
        {
            channel_manager->flush_inputs();
        }
        channel_manager_flush_timer.stop();
    }
    /**
     * Vehicles should use this to write to raw slate devices that are pushed
     * to hardware. Because unscaling will have been run at the end of the
     * previous cycle, values written here will not get overwritten by the
     * Unscaler.
     */
    void FtRuntime::dispatch_nonsynced_pre_firmware_comm() {}
    /**
     * Vehicles should use this method to generate any data that needs to
     * be written before the data sharing algorithm is run.
     */
    void FtRuntime::dispatch_nonsynced_pre_data_sharing() {}
    /**
     * Vehicles should use this method to do any processing that needs to
     * happen immediately after the data sharing algorithm is run.
     */
    void FtRuntime::dispatch_nonsynced_post_data_sharing() {}
    /**
     * Vehicles should use this method to dispatch event sources immediately
     * prior to telemetry generation.
     */
    void FtRuntime::dispatch_nonsynced_pre_telemetry() {}
    /**
     * Vehicles should override this method to provide the appropriate
     * telemetry file name for this role + instance.
     *
     * @return The name of the telemetry config file.
     */
    std::string FtRuntime::get_telemetry_config_file_name() const
    {
        return TelemetryRelay::default_config_key;
    }
    /**
     * Vehicles should override this method to specify telemetry groups that
     * should have the telem_mux_strings flag added to their flows.
     *
     * If a non-empty set is returned, all telemetry belonging to muxed groups
     * must be synced across all strings of the computer. This is enforced in
     * Slate telemetry by only passing the control Slate to the telemetry task
     * if muxing is enabled.
     *
     * Additionally, muxed telemetry channels must produce the same BWP
     * telemetry format header across strings (excluding annotations which may
     * differ in fragment but not CRC). This is enforced in Slate telemetry by
     * checking that muxed channel contain no macroexpansions that would cause
     * the resulting name to be different on different strings: [S, L, R, SELF].
     *
     * @return A set of groups to add the muxing flag to.
     */
    std::set<telem_group_t> FtRuntime::get_muxed_telemetry_groups() const
    {
        return {};
    }
    /**
     * Vehicles should override this method to provide the control code with
     * the appropriate inst for this node. This is usually just the inst that
     * the runtime is configured to use.
     *
     * @return The inst to pass to the control code.
     */
    int FtRuntime::get_control_inst() const { return ident.inst; }
    /**
     * Vehicles should override this method to set the number of local slate
     * reflection slots which will be created. By default, there are no slots.
     *
     * @return The number of local slate reflection slots to create.
     */
    size_t FtRuntime::get_num_local_reflect_slots() const { return 0u; }
    /**
     * Vehicles should override this method to instruct the Telemetry Relay to
     * create only one connection per telemetry destination. Doing so can save a
     * substantial number of file descriptors.
     */
    bool FtRuntime::should_telem_relay_aggregate_dest_connections() const
    {
        return false;
    }
    /**
     * Creates the whole peer-to-peer input sharing system.
     *
     * @param share_time The amount of time to spend in the share phase of
     *                   the data sharing algorithm.
     * @param reshare_time The amount of time to spend in the re-share phase
     *                     of the data sharing algorithm.
     *
     * @return True on success.
     */
    bool FtRuntime::create_input_sharing_system(const nano_t share_time,
                                                const nano_t reshare_time)
    {
        SacAbortIf(is_init, false);
        /*
         * Create the NodeIoManager to get the data. If we are using output
         * synchronization on this process, we do not pull output data from the
         * channel manager so we can shunt its output data into the output
         * comparator instead.
         */
        Handle<FdBag> fds_input(new FdBag);
        SacAbortIfNot(fds_input, false);
        SacAbortIfNot(node_io_manager.assume_ownership(new NodeIoManager),
                      false);
        SacAbortIfNot(node_io_manager->init(slate_local.sub_slate("node_io"),
                                            node_configs, ident, upkeep_list,
                                            *fds_input, eloop.fds),
                      false);
        if (!props.use_output_sync)
        {
            SacAbortIfNot(node_io_manager->init_output_sources(channel_manager),
                          false);
        }
        /*
         * Create the data sharer to cross-strap data and populate FT
         * channels.
         */
        /*
         * Create the sharing and re-sharing connections between the strings of
         * the controller. Note that we rely on FtSimpleDataSharer to hold onto
         * these handles at least as long as it holds onto the channels created
         * below, since those channels have references to these FdBags.
         */
        Handle<FdBag> fds_share(new FdBag);
        SacAbortIfNot(fds_share, false);
        Handle<FdBag> fds_reshare(new FdBag);
        SacAbortIfNot(fds_reshare, false);
        Handle<UdpConnection> left_share_conn;
        Handle<UdpConnection> right_share_conn;
        Handle<UdpConnection> left_reshare_conn;
        Handle<UdpConnection> right_reshare_conn;
        SacAbortIfNot(FtSimpleDataSharer::create_sharing_connections(
                          node_configs, ident, service_directory(), upkeep_list,
                          *fds_share, *fds_reshare, left_share_conn,
                          right_share_conn, left_reshare_conn,
                          right_reshare_conn),
                      false);
        /*
         * Instantiate the data sharer and connect it to the data_sig of the
         * NodeIoManager.
         */
        SacAbortIfNot(data_sharer.assume_ownership(new FtSimpleDataSharer(
                          eloop.clock, node_io_manager->data_sig,
                          should_overlap_reshare_and_input_writes())),
                      false);
        SacAbortIfNeq(ident.string.length(), 1, false);
        SacAbortIfNot(
            data_sharer->init(slate_local, ident.role_inst, ident.string[0],
                              node_configs, keychain, fds_input, fds_share,
                              fds_reshare, left_share_conn, right_share_conn,
                              left_reshare_conn, right_reshare_conn, share_time,
                              reshare_time, compute_data_sharing_input_crc),
            false);
        /*
         * Connect the data sharer's output signal to the channel manager
         * unless this process is using output synchronization, in which case
         * we leave the signal unconnected so we can later attach it to the
         * input forwarder and unsigner.
         */
        if (!props.use_output_sync)
        {
            SacAbortIfNot(data_sharer->shared_data_sig.connect(make_slot(
                              *channel_manager, &FtChannelManager::read_input)),
                          false);
        }
        if (should_send_syncs_during_reshare_phase())
        {
            SacAbortIfNot(data_sharer->reshare_phase_sig.connect(
                              make_slot(*ft_sync, &FtSync::send_syncs)),
                          false);
        }
        return true;
    }
    /**
     * Creates the local hardware communication system.
     *
     * @return True on success.
     */
    bool FtRuntime::create_adc_system()
    {
        SacAbortIf(is_init, false);
        /*
         * Create the raw slates.
         */
        SlateBuilder slate_local_raw = slate_local.sub_slate("raw");
        SlateBuilder slate_control_raw_read_create =
            slate_control_read_create.sub_slate("raw");
        /*
         * Create the firmware interface.
         */
        const double firmware_update_delay =
            cmd.get_fp("firmware_update_delay");
        /*
         * Create the FirmwareComm hardware classes.
         */
        SacAbortIfNot(firmware_comm.assume_ownership(
                          new FirmwareComm(smoketest_config.get_init_only())),
                      false);
        SacAbortIfNot(
            firmware_comm->init(
                ident, configs, firmware_update_delay, use_sensor_prefixes,
                slate_local, slate_control_read_create, slate_local_raw,
                slate_control_raw_read_create,
                slate_local.sub_slate("firmware_comm"), adc_boards_info),
            false);
        /*
         * Scale up local inputs (other than ADC channels). This is done to
         * allow telemetry to access boolean sensors and DIOs. When other
         * sensors are telemetered, they automatically pull from the
         * corresponding element in the raw slate, but booleans are pulled from
         * the scaled slate.
         */
        SacAbortIfNot(create_adc_scaler(slate_local_raw, slate_local,
                                        false /* scale_ad_banks */,
                                        adc_scaler_local),
                      false);
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
    bool
    FtRuntime::create_local_telemetry_connections(TelemetryRelayRuntime &relay)
    {
        return true;
    }
    /**
     * Create the FtBootstrapper, a simple state machine that will walk the
     * time synchronization, output synchronization, and input synchronization
     * components through the synchronization process.
     *
     * @return True on success.
     */
    bool FtRuntime::create_bootstrapper_system()
    {
        SacAbortIfNot(create_bootstrapper(bootstrapper), false);
        SacAbortIfNot(
            FtBootstrapper::populate_enums(enum_registry, local_enum_prefix()),
            false);
        /*
         * FtSync should clear its counters upon the establishment of time
         * synchronization, and we should reset all counters when we are fully
         * synchronized.
         */
        SacAbortIfNot(ft_sync, false);
        SacAbortIfNot(bootstrapper->time_sync_established_sig.connect(
                          make_slot(*ft_sync, &FtSync::reset_counters)),
                      false);
        SacAbortIfNot(bootstrapper->full_sync_established_sig.connect(
                          make_slot(*this, &FtRuntime::reset_counters)),
                      false);
        return true;
    }
    /**
     * Create the non-synced command platform.
     *
     * @return True on success.
     */
    bool FtRuntime::create_nonsynced_command_platform()
    {
        SacAbortIf(is_init, false);
        /*
         * Create a name that can be used for the dispatcher.
         */
        const std::string dispatcher_name = "nonsynced_gnd_cmd";
        /*
         * Get the input channels.
         */
        std::vector<Handle<DgramChannel>> inputs;
        SacAbortIfNot(external_command_get_default_nonsynced_inputs(
                          node_configs, ident, upkeep_list, eloop.fds, inputs),
                      false);
        /*
         * Specify a non-synced Slate shard in the calls below.
         */
        const slate_shard_t shard = shard_nonsync;
        /*
         * Create the command deframer.
         */
        Handle<ExternalCommandDeframer> deframer(
            new ExternalCommandDeframerNull());
        SacAbortIfNot(deframer, false);
        /*
         * Create the command filters.
         */
        Handle<ExternalCommandTimeFilter> time_filter;
        Handle<ExternalCommandFilter> cmd_filter;
        SacAbortIfNot(create_command_filters(time_filter, cmd_filter), false);
        SacAbortIfNot(time_filter, false);
        SacAbortIfNot(cmd_filter, false);
        /*
         * The non-synced command system has no state machine handler, so it
         * doesn't need an armer either.
         */
        Handle<ExternalCommandArmerNull> armer(new ExternalCommandArmerNull());
        SacAbortIfNot(armer, false);
        /*
         * Create the command handlers.
         */
        ext_cmd_handler_v handlers;
        /*
         * Create the slate command handler.
         */
        SacAbortIfNot(slate_command_interface_local.assume_ownership(
                          new SlateCommandInterface),
                      false);
        Handle<ExternalCommandSlateHandler> cmd_slate_handler(
            new ExternalCommandSlateHandler);
        SacAbortIfNot(cmd_slate_handler, false);
        SacAbortIfNot(cmd_slate_handler->init(slate_command_interface_local),
                      false);
        handlers.push_back(cmd_slate_handler);
        /*
         * Create and initialize the reflection manager. This must be done
         * after the slate command handler is created.
         */
        SacAbortIfNot(ReflectionManager::create(get_num_local_reflect_slots(),
                                                slate_local, shard_nonsync,
                                                "local_" /* prefix */,
                                                slate_command_interface_local,
                                                reflection_manager_local),
                      false);
        SacAbortIfNot(reflection_manager_local, false);
        /*
         * Create and initialize the reflection command handler. This must be
         * done after the reflection manager is created.
         */
        Handle<ExternalCommandReflectionHandler> reflection_handler;
        SacAbortIfNot(ExternalCommandReflectionHandler::create(
                          reflection_manager_local, reflection_handler),
                      false);
        SacAbortIfNot(reflection_handler, false);
        handlers.push_back(reflection_handler);
        /*
         * Create the output channels.
         */
        std::map<std::string, Handle<Channel>> outputs;
        SacAbortIfNot(external_command_get_default_nonsynced_outputs(
                          node_configs, upkeep_list, eloop.fds, outputs),
                      false);
        /*
         * Create and initialize the command dispatcher.
         */
        SacAbortIfNot(gnd_cmd_dispatcher_nonsynced.assume_ownership(
                          new ExternalCommandDispatcher(
                              dispatcher_name, eloop.clock, ext_cmd_params)),
                      false);
        SacAbortIfNot(gnd_cmd_dispatcher_nonsynced->init(
                          slate_local, ident.node_name, shard, inputs, deframer,
                          time_filter, cmd_filter, armer, handlers, outputs),
                      false);
        /*
         * Create some devices that can be used to test commanding.
         */
        slate_element_t element_id = 0;
        SacAbortIfNot(
            slate_local.create_element<INT64>("nonsynced_null_int", 0ll, shard,
                                              slate_private, element_id),
            false);
        SacAbortIfNot(slate_local.create_element<INT32>("nonsynced_null_int32",
                                                        0, shard, slate_private,
                                                        element_id),
                      false);
        SacAbortIfNot(
            slate_local.create_element<double>("nonsynced_null_fp", 0.0, shard,
                                               slate_private, element_id),
            false);
        return true;
    }
    /**
     * Create an AdcScaler.
     *
     * @param raw Read raw data from here.
     * @param scaled Write scaled data to here.
     * @param scale_ad_banks If false, this will skip the expensive AD banks.
     * @param[out] scaler Returns the constructed scaler.
     *
     * @return True on success.
     */
    bool FtRuntime::create_adc_scaler(SlateBuilder raw, SlateBuilder scaled,
                                      const bool scale_ad_banks,
                                      Handle<AdcScaler> &scaler)
    {
        SacAbortIf(scaler, false);
        bank_select_t bank_select = {};
        SacAbortIfNot(AdcScaler::select_all_banks(bank_select), false);
        if (!scale_ad_banks)
        {
            bank_select[ad_dev_t] = bank_select_never;
        }
        SacAbortIfNot(scaler.assume_ownership(new AdcScaler), false);
        SacAbortIfNot(scaler->init(raw, scaled, adc_boards_info, bank_select),
                      false);
        return true;
    }
    /**
     * Returns the name of the local slate.
     *
     * @return The name of the local slate.
     */
    std::string FtRuntime::local_slate_name() const { return ident.node_name; }
    /**
     * Returns the name of the control slate.
     *
     * @return The name of the control slate.
     */
    std::string FtRuntime::control_slate_name() const
    {
        return ident.control_node_name;
    }
    /**
     * Returns the name of the slate receiving the slate-shared data from
     * a given string.
     *
     * @return the name of the slate receiving the slate-shared data from
     *         a given string.
     */
    std::string FtRuntime::shared_slate_name(const char string) const
    {
        return ident.role_inst + string;
    }
    /**
     * Returns the name of the median slate.
     *
     * @return The name of the median slate.
     */
    std::string FtRuntime::median_slate_name() const { return ""; }
    /**
     * Returns the prefix that will be used by subsystems when registering
     * their devices with the enum registry. The prefix returned is essentially
     * the path difference between the root of telemetry, and the root of
     * local_slate (which is passed to the subsystems' init methods).
     *
     * @return The enum prefix.
     */
    std::string FtRuntime::local_enum_prefix() const
    {
        return local_slate_name() + ".";
    }
    /**
     * Vehicles should override this method to configure whether to overlap the
     * reshare phase with writing to the input channels.
     *
     * @return True to overlap reshare phase and writing to input channels.
     */
    bool FtRuntime::should_overlap_reshare_and_input_writes() const
    {
        return false;
    }
    /*
     * Vehicles should override this method to configure whether to send FT
     * syncs during the data sharing reshare phase.
     *
     * @return True to send FT syncs during the data sharing reshare phase.
     */
    bool FtRuntime::should_send_syncs_during_reshare_phase() const
    {
        return false;
    }
    /**
     * Handle our local slate sharing messages.
     *
     * We will pretend that this is input from the node IO manager and signal
     * it out for data sharing.
     *
     * @param channel Dgram channel with the messages.
     * @param node_id The node ID from which we will claim the data was
     *                originated.
     *
     * @return True on success.
     */
    bool FtRuntime::handle_local_share_read(DgramChannel &channel,
                                            node_id_t node_id)
    {
        SacAbortIfNot(is_init, false);
        SacAbortIfNot(node_io_manager, false);
        while (channel.dgrams_avail())
        {
            DgramData dgram = channel.get_dgram();
            SacIfNot(node_io_manager->data_sig.emit(node_id, dgram.data()));
            SacAbortIfNot(dgram.pop(), false);
        }
        return true;
    }
    /**
     * Reset the counters of all components.
     *
     * @return True on success.
     */
    bool FtRuntime::reset_counters() RUNTIME
    {
        /*
         * Reset the counters of all components.
         */
        ft_sync->reset_counters();
        if (props.use_input_sync)
        {
            data_sharer->reset_counters();
        }
        if (props.use_slate_syncer)
        {
            slate_syncer->clear_counters();
        }
        if (props.use_firmware_comm)
        {
            firmware_comm->reset_counters();
        }
        if (gnd_cmd_dispatcher_nonsynced)
        {
            gnd_cmd_dispatcher_nonsynced->reset_counters();
        }
        /*
         * Reset the latching information in our timers.
         */
        slate_syncer_replace_timer.reset_counters();
        firmware_comm_timer.reset_counters();
        pre_data_sharing_timer.reset_counters();
        data_sharing_timer.reset_counters();
        post_data_sharing_timer.reset_counters();
        pre_telemetry_timer.reset_counters();
        telemetry_timer.reset_counters();
        slate_syncer_share_timer.reset_counters();
        slate_roll_frame_timer.reset_counters();
        channel_manager_flush_timer.reset_counters();
        eloop_dispatch_select_timer.reset_counters();
        /*
         * Allow the child class to reset the counters of any of its node-
         * specific components.
         */
        reset_runtime_counters();
        return true;
    }
    /**
     * Vehicles should override this method to clear the counters of their
     * components.
     */
    void FtRuntime::reset_runtime_counters() RUNTIME {}
} /* end namespace Drone */