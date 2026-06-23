/**
 * @author Nathan DeVries
 * @date   2019/05/17
 */
#include "src/flight/sat/all/flight-computer/DroneGncControl.h"
#include "src/bullwinkle/all/AlertManagerControl.h"
#include "src/bullwinkle/all/ExternalCommandGncHandler.h"
#include "src/bullwinkle/all/ExternalCommandMultiSlateHandler.h"
#include "src/bullwinkle/all/ExternalCommandReflectionHandler.h"
#include "src/bullwinkle/all/core/fswtime.h"
#include "src/flight/common/all/ExternalCommandFilterCommon.h"
#include "src/flight/sat/all/common/control/DronePermanentFailures.h"
#include "src/flight/sat/all/common/control/DroneSuppressedAlertsConfig.h"
#include "src/flight/sat/all/common/control/TimedCommandQueue.h"
#include "src/flight/sat/all/gnc/DroneRegisterFlightComputerGnc.h"
namespace
{
    /**
     * Maximum capacity of the timed command queue.
     */
    constexpr size_t timed_command_queue_capacity = 50;
    /**
     * Filename for failed hardware event sequence config.
     */
    const std::string failed_hardware_seq_filename = "permanent_failure_seqs";
    /**
     * Compression level to pass to save_snapshot() for state registry dumps.
     */
    const std::string dump_file_compression_level = "wb6";
    /**
     * Permissible staleness when evaluating a timed command, beyond which a
     * timed command will be rejected for being in the past.
     *
     * Rationale: 2x control cycle, plus a little extra.
     */
    constexpr Drone::nano_t command_staleness_threshold =
        12 * Drone::ns_per_s;
} // namespace
namespace Drone
{
    /**
     * Constructor.
     */
    DroneGncControl::DroneGncControl(
        const std::string &_state_registry_dump_file)
        : BasicControl(ExternalCommandDispatcher::config_params_t(
                           Satellite::ext_cmd_dedup_capacity_synced,
                           Satellite::ext_cmd_dedup_expiration,
                           Satellite::ext_cmd_proxy_watchdog_timeout,
                           satgnc_control_period,
                           Satellite::ext_cmd_proxy_watchdog_enabled),
                       "telemetry"),
          state_registry(), gnc_component_factory(), gnc(), sharer_receivers(),
          sharer_combiners(), sr_conditional_interface_manager(),
          sr_interface_manager(), satfc_comms_mapper(), satfc_command_spammer(),
          satfc_cmd_tok(), old_satfc_cmd_tok(),
          reference_trajectory_sequence_number_tok(),
          last_received_reference_trajectory_sequence_number_tok(),
          state_machine_ever_dispatched_tok(), gnc_command_handler(),
          multi_command_handler(), timed_command_queue(), reflection_manager(),
          command_filter_synced(), alert_manager(), calibration_status_tok(),
          failed_hardware_seq_handler(), state_registry_dumps_enabled_tok(),
          cola_burn_monitor(),
          state_registry_dump_file(_state_registry_dump_file)
    {}
    /**
     * Returns the identity to use when claiming Slate sharers. This process is
     * co-located on the satfc, so we need to return our host node's identity.
     *
     * @return The identity to use when claiming Slate sharers.
     */
    ControlNodeIdentity DroneGncControl::get_slate_sharing_identity() const
    {
        ControlNodeIdentity slate_sharing_ident;
        FswAssert(slate_sharing_ident.init("satfc", 1));
        return slate_sharing_ident;
    }
    /**
     * Perform initial bring-up of the timestamp synchronizer.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_initial_systems()
    {
        FswAbortIf(is_init, false);
        /*
         * Initialize the telemetry timestamp synchronizer. Since this
         * system sends timestamps to itself on the next cycle, accept
         * timestamps which are up to one cycle old.
         */
        const nano_t max_master_age = 1 * satgnc_control_period;
        FswAbortIfNot(timestamp_syncer.assume_ownership(
                          new TimestampSynchronizer(clock, max_master_age)),
                      false);
        /*
         * Initialize the synchronized_timestamp token early in vehicle
         * bring-up, so other components can bind to the token.
         */
        FswAbortIfNot(
            timestamp_syncer->init_synchronized_timestamp(slate_control),
            false);
        return true;
    }
    /**
     * Creates the sending side of remote-sharing systems like the timestamp
     * synchronizer. This runs after the creation of the control systems so
     * it has access to the elements that they may create.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_shared_sender_systems()
    {
        FswAbortIf(is_init, false);
        /*
         * Initialize the rest of TimestampSynchronizer now that local slate
         * sharing has been initialized and median elements are available.
         */
        {
            /*
             * The GNC control process is its own timestamp master.
             */
            str_v followers;
            followers.push_back("");
            /*
             * This node is the master, so it should create master slate
             * elements rather than bind to them.
             */
            const bool create_master_elems = true;
            /*
             * Since this node is the master, the master slate is this
             * node's control slate.
             */
            SlateBuilder slate_master = slate_control;
            FswAbortIfNot(timestamp_syncer->init(
                              slate_control, slate_shared, slate_control,
                              slate_master, followers, create_master_elems),
                          false);
        }
        return true;
    }
    /**
     * Create systems for receiving data from other nodes or processes.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_shared_receiver_systems()
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(setup_conjunction_receiver(), false);
        FswAbortIfNot(setup_low_velo_conjunction_receiver(), false);
        FswAbortIfNot(setup_reference_trajectory_receiver(), false);
        FswAbortIfNot(setup_space_weather_receiver(), false);
        return true;
    }
    /**
     * Execute all synchronized vehicle control code.
     */
    void DroneGncControl::execute_synced() RUNTIME
    {
        if (FswIfNot(is_init))
        {
            return;
        }
        const nano_t control_time = clock.control_time();
        /*
         * Combine local shared data.
         */
        FswIfNot(slate_combiner_control->combine());
        /*
         * Dispatch the remote slate combiners.
         */
        for (const Handle<SlateCombiner> &slate_combiner : sharer_combiners)
        {
            FswIfNot(slate_combiner->combine());
        }
        /*
         * Set the Clock's telemetry timestamp for this control cycle and
         * publish the next cycle's timestamp to followers. This must be done
         * after local slate sharing and remote slate receiving, but before
         * remote slate sending.
         *
         * BasicControl subclasses are responsible for creating and
         * initializing TimestampSynchronizer appropriately.
         */
        FswIfNot(timestamp_syncer->dispatch());
        /*
         * Dispatch failed hardware event sequence handler.
         */
        FswIfNot(failed_hardware_seq_handler->dispatch());
        /*
         * Handle commands from the main control process first.
         * We skip this the first cycle (before the state machine has ever been
         * dispatched) to avoid commanding while in "init".
         */
        if (slate[state_machine_ever_dispatched_tok])
        {
            FswIfNot(handle_sm_commands());
        }
        satfc_command_spammer->dispatch(control_time);
        /*
         * Dispatch synced commands and update the watchdog and
         * disconnect spammer.
         */
        FswIfNot(ground_cmd_dispatcher_synced->dispatch());
        disconnect_command_spammer->dispatch(control_time);
        /*
         * Dispatch any time triggered commands.
         */
        FswIfNot(timed_command_queue->dispatch());
        /*
         * Dispatch the StateRegistryInput interfaces.
         */
        FswIfNot(sr_interface_manager->dispatch_inputs());
        /*
         * If we have a valid reference trajectory dispatch conditional
         * StateRegistryInput interfaces.
         */
        if (slate[reference_trajectory_sequence_number_tok] >
            slate[last_received_reference_trajectory_sequence_number_tok])
        {
            FswIfNot(sr_conditional_interface_manager->dispatch_inputs());
            slate[last_received_reference_trajectory_sequence_number_tok] =
                slate[reference_trajectory_sequence_number_tok];
        }
        /*
         * Dispatch the state machine.
         */
        state_machine->dispatch(control_time);
        slate[state_machine_ever_dispatched_tok] = true;
        /*
         * Dispatch the StateRegistryOutput interfaces.
         */
        FswIfNot(sr_interface_manager->dispatch_outputs());
        /*
         * Run satfc_comms_mapper late in cycle so that next cycle will have
         * fresh data to share to the vehicle control process.
         */
        FswIfNot(satfc_comms_mapper->dispatch());
        /*
         * The AlertManagerControl must be dispatched after anything that may
         * signal an alert. It maintains state in the synced slate shard, so it
         * should be part of dispatch_synced.
         */
        alert_manager->dispatch();
        /*
         * Dispatch the COLA burn monitor if dumping the state registry to a
         * file is enabled.
         */
        if (slate[state_registry_dumps_enabled_tok])
        {
            cola_burn_monitor->dispatch(control_time);
        }
        /*
         * Dispatch the reflection manager for control devices.
         *
         * This needs to run after all other control code so that
         * the latest value of each reflected device is copied into the
         * slot.
         */
        reflection_manager->dispatch();
    }
    /**
     * Creates the guidance, navigation, and control systems.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_gnc_system()
    {
        FswAbortIf(is_init, false);
        /*
         * Register components.
         */
        FswAbortIfNot(register_satfc_gnc_components(), false);
        /*
         * Create the StateRegistry.
         */
        FswAbortIfNot(state_registry.assume_ownership(
                          new StateRegistry(slate_control.sub_slate("gnc"))),
                      false);
        const std::string allowed_variables_file =
            "satgnc_state_registry_variables";
        FswAbortIfNot(state_registry->init(configs, alert_provider,
                                           allowed_variables_file),
                      false);
        /*
         * GNC factory.
         */
        FswAbortIfNot(
            gnc_component_factory.assume_ownership(
                new GncComponentFactory(clock, slate_control, *state_registry,
                                        *channel_manager, configs, cmd_table)),
            false);
        FswAbortIfNot(gnc_component_factory->init("mission_gnc_components"),
                      false);
        /*
         * GNC controller.
         */
        FswAbortIfNot(gnc.assume_ownership(new GncController(
                          "gnc_controller", satgnc_control_period)),
                      false);
        FswAbortIfNot(gnc->init(slate_control, configs, gnc_component_factory,
                                "mission_gnc_controller", ""),
                      false);
        /*
         * Make the state machine listen to GNC transition requests.
         */
        FswAbortIfNot(gnc->cmd_sig.connect(
                          make_slot(*state_machine, &StateMachine::handle_cmd)),
                      false);
        /*
         * Create the state registry interfaces.
         */
        FswAbortIf(sr_interface_manager, false);
        FswAbortIfNot(
            sr_interface_manager = StateRegistrySlateInterfaceManager::create(
                state_registry, slate_control, configs, "sr_interface_manager"),
            false);
        /*
         * Create the conditional state registry interfaces.
         */
        FswAbortIf(sr_conditional_interface_manager, false);
        FswAbortIfNot(sr_conditional_interface_manager =
                          StateRegistrySlateInterfaceManager::create(
                              state_registry, slate_control, configs,
                              "sr_conditional_interface_manager"),
                      false);
        /*
         * No futher changes to the GNC state allowed.
         */
        FswAbortIfNot(state_registry->lock(), false);
        /*
         * Create devices for StateRegistry.
         */
        FswAbortIfNot(state_registry->create_devices(), false);
        /*
         * Initialize StateRegistry alarms.
         */
        FswAbortIfNot(state_registry->init_alarms(), false);
        /*
         * Initialize a monitor that dumps the state registry to a file.
         */
        SlateBuilder gnc_subslate = slate_control.sub_slate("gnc");
        cola_burn_monitor =
            ColaBurnMonitor::create(slate_control, gnc_subslate);
        FswAbortIfNot(cola_burn_monitor, false);
        if (!state_registry_dump_file.empty())
        {
            /*
             * To ensure atomic writes, save_snapshot writes first to this tmp
             * file, then moves it to the dump file.
             */
            const std::string tmp_file = state_registry_dump_file + ".tmp";
            /*
             * Wire up the ColaBurnMonitor to do the state registry dump.
             */
            FswAbortIfNot(
                cola_burn_monitor->cola_event_sig.connect(slot_bind(
                    make_slot(*state_registry, &StateRegistry::save_snapshot),
                    state_registry_dump_file, tmp_file,
                    dump_file_compression_level)),
                false);
        }
        gnc_component_factory->clear_config_cache();
        return true;
    }
    /**
     * Creates unified control systems.
     *
     * @param[in, out] cycle_timer_requests The set of all CycleTimer to
     * create.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_control_systems(
        cycle_timer_name_s &cycle_timer_requests)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(create_gnc_system(), false);
        /*
         * Spam commands from the main control process into the state machine.
         */
        FswAbortIfNot(satfc_command_spammer.assume_ownership(new CommandSpammer(
                          clock, state_machine,
                          ident.role_inst + "_satfc_command_spammer")),
                      false);
        FswAbortIfNot(
            satfc_command_spammer->init(slate_control, control_period), false);
        /*
         * Bind to the state machine command.
         */
        FswAbortIfNot(
            slate_control.bind("satfc1.state_machine_command", satfc_cmd_tok),
            false);
        FswAbortIfNot(slate_control.create("old_state_machine_command", 0,
                                           shard_sync, slate_read_only,
                                           old_satfc_cmd_tok),
                      false);
        FswAbortIfNot(slate_control.create("state_machine_ever_dispatched", 0,
                                           shard_sync, slate_read_only,
                                           state_machine_ever_dispatched_tok),
                      false);
        /*
         * Create the calibration parsed token.
         */
        FswAbortIfNot(slate_control.create("calibration_status",
                                           calibration_status_parse_succeeded,
                                           shard_sync, slate_read_only,
                                           calibration_status_tok),
                      false);
        /*
         * Create feature flag for state registry dumps.
         */
        FswAbortIfNot(slate_control.create("state_registry_dumps_enabled", true,
                                           shard_sync, slate_read_write,
                                           state_registry_dumps_enabled_tok),
                      false);
        /*
         * Create failed hardware handler.
         */
        FswAbortIfNot(failed_hardware_seq_handler.assume_ownership(
                          new TriggeredEventSequenceHandler),
                      false);
        FswAbortIfNot(failed_hardware_seq_handler->init_storage(
                          *alert_provider, configs, slate_control,
                          failed_hardware_seq_filename, ident),
                      false);
        return true;
    }
    /**
     * Adds post-transition control tasks to the state machine.
     *
     * @Return True on success.
     */
    bool DroneGncControl::build_state_machine_post_transition()
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(state_machine, false);
        /*
         * GNC mode depends on the state machine state.
         */
        FswAbortIfNot(state_machine->add_ctask("gnc", gnc), false);
        return true;
    }
    /**
     * Create a SlateSharerReceiver for receiving the conjunction information
     * from the ConjunctionDataClient.
     *
     * @return True on success.
     */
    bool DroneGncControl::setup_conjunction_receiver()
    {
        FswAbortIf(is_init, false);
        const sharer_config_v receiver_config(
            1, sharer_config_t("satellite_fleet_client_conjunction_to_satgnc",
                               ""));
        FswAbortIfNot(
            setup_remote_node_receiver(
                "fleet_client_conjunction1", receiver_config, "fleet_client1",
                "conjunction_data_client.last_cdm_fetch_ns", nano_t_max, true),
            false);
        return true;
    }
    /**
     * Create a SlateSharerReceiver for receiving the low velocity conjunction
     * information from the LowVeloConjunctionDataClient.
     *
     * @return True on success.
     */
    bool DroneGncControl::setup_low_velo_conjunction_receiver()
    {
        FswAbortIf(is_init, false);
        const sharer_config_v receiver_config(
            1,
            sharer_config_t(
                "satellite_fleet_client_low_velo_conjunction_to_satgnc", ""));
        FswAbortIfNot(setup_remote_node_receiver(
                          "fleet_client_low_velo_conjunction1", receiver_config,
                          "fleet_client_low_velo_conjunction1",
                          "low_velo_conjunction_data_client.last_traj_fetch_ns",
                          nano_t_max, true),
                      false);
        return true;
    }
    /**
     * Create a SlateSharerReceiver for receiving the reference trajectory
     * information from the SatelliteReferenceTrajectoryClient.
     *
     * @return True on success.
     */
    bool DroneGncControl::setup_reference_trajectory_receiver()
    {
        FswAbortIf(is_init, false);
        const sharer_config_v reference_config(
            1, sharer_config_t("reference_trajectory_to_satgnc", ""));
        FswAbortIfNot(setup_remote_node_receiver(
                          "fleet_client_reference_trajectory1",
                          reference_config,
                          "fleet_client_reference_trajectory1",
                          "receiver.sequence_number", nano_t_max, true),
                      false);
        return true;
    }
    /**
     * Create a SlateSharerReceiver for receiving the space weather
     * information from the SpaceWeatherClient.
     *
     * @return True on success.
     */
    bool DroneGncControl::setup_space_weather_receiver()
    {
        FswAbortIf(is_init, false);
        const sharer_config_v receiver_config(
            1, sharer_config_t("satellite_fleet_client_space_weather_to_satgnc",
                               ""));
        FswAbortIfNot(setup_remote_node_receiver(
                          "fleet_client_space_weather1", receiver_config,
                          "fleet_client_space_weather1",
                          "space_weather_client.last_space_weather_fetch_ns",
                          nano_t_max, true),
                      false);
        return true;
    }
    /*
     * Sets up a slate sharing receiver and combiner for a remote node.
     *
     * @param component_name The name of the node we're receiving from.
     * @param receiver_config The sharer_config_v for this receiver.
     * @param subslate_name The name of the subslate to use for received data.
     * @param freshness_name The name of the token to use for freshness of the
     * combiner.
     * @param cycle_delay The cycle_delay to use for the receiver.
     * @param downselect Whether to use the downselect mode for the combiner.
     *
     * @return True on success.
     */
    bool DroneGncControl::setup_remote_node_receiver(
        const std::string &component_name,
        const sharer_config_v &receiver_config,
        const std::string &subslate_name, const std::string &freshness_name,
        const nano_t cycle_delay, const bool downselect)
    {
        /*
         * We expect to receive a packet from each remote node by receiving
         * either the same packet (multicast) on each of the strings or a
         * different packet with the exact same contents (unicast). To ensure
         * everyone has a consistent view of what data was received on each of
         * the strings, each one will set up a receiver for each string and data
         * sharing will ensure that the copies received on each string are
         * shared. These individual copies will be received into
         * satgnc1x.satgnc1[abc].subslate_name.
         */
        slate_combiner_in_v combiner_in;
        for (size_t i = 0; i < Satellite::num_fc_strings; i++)
        {
            const char string = static_cast<char>('a' + i);
            SlateBuilder receiver_subslate =
                slate_control.sub_slate(ident.role_inst + string)
                    .sub_slate(subslate_name);
            Handle<SlateSharerReceiver> receiver;
            Handle<DgramChannel> in;
            FswAbortIfNot(
                channel_manager->get_dgram_input(component_name + string, in),
                false);
            FswAbortIfNot(receiver.assume_ownership(
                              new SlateSharerReceiver(clock, cycle_delay)),
                          false);
            FswAbortIfNot(receiver->init("receiver", receiver_subslate,
                                         shard_sync, configs, receiver_config,
                                         in, sharer_create),
                          false);
            sharer_receivers.push_back(receiver);
            const std::string combiner_string(1, string);
            combiner_in.push_back(
                slate_combiner_in_t(receiver_subslate, combiner_string));
        }
        /*
         * We finally create a slate combiner to collate those slates into the
         * base control sub-slate, assumed to be satgnc1x.subslate_name. All
         * control will operate on this new, combined sub-slate, ensuring that
         * the gnc computers will not desync in the event of a dropped packet
         */
        Handle<SlateCombiner> combiner;
        FswAbortIfNot(combiner.assume_ownership(new SlateCombiner), false);
        FswAbortIfNot(combiner->init(configs, freshness_name, combiner_in,
                                     slate_control.sub_slate(subslate_name),
                                     receiver_config, false, /* bind_outputs */
                                     "combiner",             /* combiner_name */
                                     slate_read_only,        /* access */
                                     downselect,             /* _downselect */
                                     false),                 /* bind_metrics */
                      false);
        sharer_combiners.push_back(combiner);
        return true;
    }
    /**
     * Add various enumerations to the enum registry.
     *
     * @return True on success.
     */
    bool DroneGncControl::populate_enums()
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(enum_registry, false);
        /*
         * Register GNC enums.
         */
        FswAbortIfNot(enum_registry->register_other(
                          ident.role_inst + "x.gnc",
                          gnc_component_factory->get_enum_registry()),
                      false);
        /*
         * Set up satgnc state machine commands.
         */
        {
            std::string path;
            FswAbortIfNot(slate_control.get_path(satfc_cmd_tok, path), false);
            str_v channels;
            channels.push_back(path);
            FswAbortIfNot(cmd_table.populate_enums(*enum_registry, channels),
                          false);
        }
        /*
         * Set up enum for calibration parsing.
         */
        {
            str_v channels;
            channels.push_back(ident.control_node_name + ".calibration_status");
            FswAbortIfNot(enum_registry->register_enum(
                              "calibration_status_t", calibration_status_t_sym,
                              channels, "calibration_status_"),
                          false);
        }
        return true;
    }
    /**
     * Handle state machine commands from the main control process.
     *
     * @note We only send a command if it is different from the previous
     * cycle. This is OK here because the satgnc state machine is configured
     * such that we would neve expect a different final state after sending
     * a command once vs twice.
     *
     * return True on success.
     */
    bool DroneGncControl::handle_sm_commands() RUNTIME
    {
        if (slate[satfc_cmd_tok] != 0 &&
            slate[satfc_cmd_tok] != slate[old_satfc_cmd_tok])
        {
            slate[old_satfc_cmd_tok] = slate[satfc_cmd_tok];
            vehicle_cmd_t vehicle_cmd;
            FswMsgAbortIfNot(
                cmd_table.lookup(slate[satfc_cmd_tok], vehicle_cmd), false, 100,
                "Invalid command index '%d'.\n", slate[satfc_cmd_tok]);
            FswAbortIfNot(satfc_command_spammer->handle_cmd(vehicle_cmd),
                          false);
        }
        return true;
    }
    /**
     * Vehicles should use this method to create command handlers
     * to be used by the ExternalCommandDispatcher.
     *
     * @param dispatcher_name Name of the command dispatcher (used as a base
     *                        for command handler names).
     * @param[out] handlers Command handlers.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_synced_command_handlers(
        const std::string &dispatcher_name, ext_cmd_handler_v &handlers)
    {
        FswAbortIf(is_init, false);
        /*
         * Get the default list of handlers used by BasicControl.
         */
        ext_cmd_handler_v base_handlers;
        FswAbortIfNot(BasicControl::create_synced_command_handlers(
                          dispatcher_name, base_handlers),
                      false);
        /*
         * Set up the GNC command handler.
         */
        FswAbortIfNot(gnc_command_handler.assume_ownership(
                          new ExternalCommandGncHandler(*state_registry)),
                      false);
        FswAbortIfNot(gnc_command_handler->init(), false);
        base_handlers.push_back(gnc_command_handler);
        /*
         * Set up the Multi command handler.
         */
        FswAbortIfNot(slate_command_interface_control, false);
        Handle<ExternalCommandMultiSlateHandler> multi_cmd_handler(
            new ExternalCommandMultiSlateHandler());
        FswAbortIfNot(multi_cmd_handler, false);
        FswAbortIfNot(multi_cmd_handler->init(slate_command_interface_control),
                      false);
        base_handlers.push_back(multi_cmd_handler);
        /*
         * Create and initialize the reflection manager. This must be done after
         * the slate command handler is created.
         */
        FswAbortIfNot(ReflectionManager::create(
                          Satellite::num_control_reflect_slots, slate_control,
                          shard_sync, "control_" /* prefix */,
                          slate_command_interface_control, reflection_manager),
                      false);
        FswAbortIfNot(reflection_manager, false);
        /*
         * Create and initialize the reflection command handler. This must be
         * done after the reflection manager is created.
         */
        Handle<ExternalCommandReflectionHandler> reflection_handler;
        FswAbortIfNot(ExternalCommandReflectionHandler::create(
                          reflection_manager, reflection_handler),
                      false);
        FswAbortIfNot(reflection_handler, false);
        base_handlers.push_back(reflection_handler);
        {
            /*
             * Set up the TimedCommandQueue.
             *
             * This wants its own copy of the command filter, so we init a new
             * one from the synced one.
             */
            Handle<ExternalCommandFilterCommon> common_cmd_filter(
                new ExternalCommandFilterCommon);
            FswAbortIfNot(common_cmd_filter->init(command_filter_synced),
                          false);
            FswAbortIfNot(
                timed_command_queue = TimedCommandQueue::create(
                    timed_command_queue_capacity, slate_control,
                    common_cmd_filter, base_handlers,
                    true,                         /* synchronized_time */
                    command_staleness_threshold), /* cmd_in_past_threshold
                                                   */
                false);
            handlers = base_handlers;
            handlers.push_back(timed_command_queue);
        }
        return true;
    }
    /**
     * Return the synced command filter.
     *
     * @param[out] cmd_filter The synced command filter.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_synced_command_filter(
        Handle<ExternalCommandFilter> &cmd_filter)
    {
        FswAbortIf(is_init, false);
        /*
         * Create the command filter.
         *
         * We are re-using the common ExternalCommandFilter, because it is
         * sufficiently configurable through config and isn't really vehicle
         * specific.
         */
        FswAbortIf(command_filter_synced, false);
        FswAbortIfNot(command_filter_synced.assume_ownership(
                          new CommonCommandFilter("command_filter")),
                      false);
        FswAbortIfNot(
            command_filter_synced->init_storage(
                configs, slate_control, shard_sync, true /* enabled_at_init */),
            false);
        Handle<ExternalCommandFilterCommon> common_cmd_filter(
            new ExternalCommandFilterCommon);
        FswAbortIfNot(common_cmd_filter->init(command_filter_synced), false);
        cmd_filter = common_cmd_filter;
        return true;
    }
    /**
     * Set up the AlertManager.
     *
     * @return True on success.
     */
    bool DroneGncControl::create_alert_system()
    {
        FswAbortIf(is_init, false);
        /*
         * Set up the alert system so that systems initialized after this
         * point can get the alerts they need to signal.
         *
         * Configure the AlertManager to automatically reset alerts after 24
         * hours.
         */
        FswAbortIf(alert_manager, false);
        FswAbortIfNot(alert_manager.assume_ownership(new AlertManagerControl()),
                      false);
        FswAbortIfNot(
            alert_manager->set_auto_reset_delta(24 * 60 * 60 * billion), false);
        FswAbortIfNot(alert_manager->init(
                          configs, "alert_groups", "alerts", slate_control,
                          TimestampSynchronizer::synchronized_timestamp_path),
                      false);
        /*
         * Make alerts accessible to BasicControl. We still hold a local
         * derived-type handle for convenience.
         */
        FswAbortIf(alert_provider, false);
        alert_provider = alert_manager;
        return true;
    }
    /**
     * Override BasicControl::create_basic_synced_command_platform so that we
     * can create a SlateMapper after the ExternalCommandDispatcher is created.
     */
    bool DroneGncControl::create_basic_synced_command_platform()
    {
        FswAbortIfNot(BasicControl::create_basic_synced_command_platform(),
                      false);
        /*
         * Setup a slate mapper for copying some channels from root slate
         * to a satfc1-outbound slate.
         */
        {
            FswAbortIfNot(
                satfc_comms_mapper.assume_ownership(new SlateMapper()), false);
            FswAbortIfNot(satfc_comms_mapper->init(slate_control, configs,
                                                   "satfc_comms_mapper", {""},
                                                   true, shard_sync),
                          false);
        }
        return true;
    }
    /**
     * Perform all final initialization before the slate is built.
     *
     * @return True on success.
     */
    bool DroneGncControl::pre_finalize_slate()
    {
        FswAbortIfNot(BasicControl::pre_finalize_slate(), false);
        /*
         * Finalize the alert manager. Calling get_signal_tok() after this point
         * will fail.
         */
        FswAbortIfNot(alert_manager->finalize(slate_control), false);
        /*
         * Bind to reference reference trajectory sequence number.
         */
        FswAbortIfNot(
            slate_control.bind("fleet_client_reference_trajectory1.reference_"
                               "trajectory.sequence_number",
                               reference_trajectory_sequence_number_tok),
            false);
        /*
         * Create last received reference trajectory sequence number.
         */
        FswAbortIfNot(
            slate_control.create(
                "last_received_reference_trajectory_sequence_number", 0,
                shard_sync, slate_read_write,
                last_received_reference_trajectory_sequence_number_tok),
            false);
        return true;
    }
    /**
     * Vehicles should use this method to do vehicle-specific
     * post-slate-creation setup.
     *
     * @param sudo_slate_control The super slate builder for the control slate.
     * @name cycle_timers The map of all CycleTimers that have been created.
     *
     * @return True on success.
     */
    bool DroneGncControl::finalize_control_systems(
        SlateBuilder sudo_slate_control, const cycle_timer_m &cycle_timers)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(failed_hardware_seq_handler->init(
                          sudo_slate_control, configs, *enum_registry,
                          ident.control_node_name),
                      false);
        slate[calibration_status_tok] = parse_calibration();
        FswIfNeq(slate[calibration_status_tok],
                 calibration_status_parse_succeeded);
        if (!state_registry_dump_file.empty())
        {
            /*
             * On init, write a dump to the .tmp file. This will never be
             * downlinked, but it ensures that memory usage isn't artificially
             * low before the first state registry dump.
             */
            const std::string tmp_file = state_registry_dump_file + ".tmp";
            FswAbortIfNot(state_registry->save_snapshot(
                              tmp_file, state_registry_dump_file,
                              dump_file_compression_level),
                          false);
        }
        return true;
    }
    /**
     * Finalize the command platform.
     *
     * @param sudo_slate_control The super slate builder for the control
     * slate.
     *
     * @return True on success.
     */
    bool DroneGncControl::finalize_basic_command_platform(
        SlateBuilder sudo_slate_control)
    {
        FswAbortIf(is_init, false);
        /*
         * A list of valid state machine prefixes to determine the
         * destination state machine of an incoming command. There are no
         * other state machines in DroneGncControl that we want to
         * command right now.
         */
        const str_v sm_prefixes;
        FswAbortIfNot(command_filter_synced->init(
                          configs, sudo_slate_control, cmd_table,
                          *state_registry, "satgnc_state_registry_variables",
                          "gnc" /* sr_slate_prefix */, sm_prefixes,
                          alert_manager),
                      false);
        /*
         * Initialize the Slate command interfaces.
         */
        FswAbortIfNot(slate_command_interface_control, false);
        FswAbortIfNot(slate_command_interface_control->init(sudo_slate_control),
                      false);
        return true;
    }
    /**
     * Parse the calibration used by this control process.
     *
     * @return calibration_status_t indicating the status of parsing
     * calibration.
     */
    calibration_status_t DroneGncControl::parse_calibration()
    {
        std::vector<calibration_dir_t> permanent_failures_reldirs;
        FswAbortIfNot(get_per_vehicle_calibration_reldirs(
                          "satellite",
                          /* include_manually_updated_calibration_dir */ true,
                          permanent_failures_reldirs),
                      calibration_status_vehicle_unknown);
        std::vector<calibration_dir_t> standard_calibration_reldirs;
        FswAbortIfNot(get_per_vehicle_calibration_reldirs(
                          "satellite",
                          /* include_manually_updated_calibration_dir */ false,
                          standard_calibration_reldirs),
                      calibration_status_vehicle_unknown);
        /*
         * Parse each individual calibration and take the maximum of all the
         * returned values to indicate the deepest failure.
         */
        calibration_status_t status =
            parse_permanent_failures(permanent_failures_reldirs);
        status = std::max(
            status, parse_suppressed_alerts(standard_calibration_reldirs));
        return status;
    }
    /**
     * Parse the permanent failures calibration configuration file.
     *
     * @param root_reldirs relative directories in / to look
     *                        for the calibration config
     *
     * @return calibration_status_t indicating the status of parsing
     * calibration.
     */
    calibration_status_t DroneGncControl::parse_permanent_failures(
        const std::vector<calibration_dir_t> &root_reldirs)
    {
        Configs calibrations;
        str_v failure_file_names = {"permanent_failures",
                                    "per_vehicle_config_permanent_failures"};
        calibration_status_t status = setup_per_vehicle_calibration(
            calibrations, failure_file_names, root_reldirs);
        FswAbortIfNeq(status, calibration_status_parse_succeeded, status);
        str_v config_keys = {"permanent_failures",
                             "per_vehicle_config_permanent_failures"};
        return DronePermanentFailures::run_init_sequences(
            calibrations, config_keys, failed_hardware_seq_handler);
    }
    /**
     * Parse the suppressed alerts calibration configuration file.
     *
     * @param root_reldirs relative directories in / to look
     *                        for the calibration config
     *
     * @return calibration_status_t indicating the status of parsing
     * calibration.
     */
    calibration_status_t DroneGncControl::parse_suppressed_alerts(
        const std::vector<calibration_dir_t> &root_reldirs)
    {
        Configs calibrations;
        str_v alert_file_names = {"suppressed_alerts",
                                  "per_vehicle_config_suppressed_alerts"};
        calibration_status_t status = setup_per_vehicle_calibration(
            calibrations, alert_file_names, root_reldirs);
        FswAbortIfNeq(status, calibration_status_parse_succeeded, status);
        return DroneSuppressedAlertsConfig::parse_and_apply(
            calibrations, alert_file_names, slate_control, ident);
    }
} /* namespace Drone */