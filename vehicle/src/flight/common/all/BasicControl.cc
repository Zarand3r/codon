/**
 * @author Stefan Moluf
 * @date   2013/02/07
 */
#include "src/flight/common/all/BasicControl.h"
#include "src/bullwinkle/all/ExternalCommandStateMachineHandler.h"
#include "src/bullwinkle/all/FtraceTrap.h"
#include "src/bullwinkle/all/VoidAlertProvider.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/external_command_util.h"
namespace Drone
{
    /**
     * Constructor.
     *
     * @param _ext_cmd_params The set of parameters used to configure the
     *                        control (synchronized) ExternalCommandDispatcher.
     * @param _telem_config_key Name of the telemetry config file used to create
     *                          control devices for the telemetry system.
     */
    BasicControl::BasicControl(
        const ExternalCommandDispatcher::config_params_t &_ext_cmd_params,
        const std::string &_telem_config_key)
        : ext_cmd_params(_ext_cmd_params), telem_config_key(_telem_config_key),
          clock(), configs(), is_init(false), control_period(nano_t_max),
          ident(), slate_shared(), slate_control(), slate(), keychain(),
          channel_manager(), slate_sharer_manager(), slate_sharer_scaling(),
          alarm_inhibit_tok(), reset_counters_tok(), adc_boards_info(),
          adc_powersaver(), adc_scaler_shared_a(), adc_scaler_shared_b(),
          adc_scaler_shared_c(), adc_scaler_control(), adc_unscaler_control(),
          adc_scaler_exclude_channels(), slate_receiver_a(), slate_receiver_b(),
          slate_receiver_c(), slate_combiner_control(), cmd_table(),
          state_machine(), telem_relay_control(), alarm_sequence_handler(),
          alert_provider(), slate_alarms(), ground_cmd_dispatcher_synced(),
          slate_command_interface_control(), disconnect_command_spammer(),
          enum_registry(), enumerated_bitmaps(), timestamp_syncer(),
          adc_unscale_timer("prof.adc_unscale")
    {}
    /**
     * Destructor.
     */
    BasicControl::~BasicControl() {}
    /**
     * Provide BasicControl with fundamentals regarding its identity.
     *
     * @param _control_period The period at which we control.
     * @param _configs The config file finder.
     * @param process_name The name of the process this control component is
     *                     running in.
     * @param _ident The control node identity for this component.
     * @param _slate_control The root of the control slate.
     * @param[in,out] node_configs The list of remote I/O nodes described in the
     *                             configs. BasicControl will add any additional
     *                             I/O nodes required for Slate sharing to this
     *                             list.
     *
     * @return True on success.
     */
    bool BasicControl::init_basic_identity(const nano_t _control_period,
                                           const Configs &_configs,
                                           const std::string &process_name,
                                           const ControlNodeIdentity &_ident,
                                           SlateBuilder _slate_control,
                                           FtNodeConfigList &node_configs)
    {
        FswAbortIf(is_init, false);
        control_period = _control_period;
        configs = _configs;
        ident = _ident;
        slate_control = _slate_control;
        FswAbortIfNot(slate_sharer_manager.assume_ownership(
                          new SlateSharerManagerTripleString(clock)),
                      false);
        FswAbortIfNot(slate_sharer_manager->init(
                          control_period, configs, process_name,
                          get_slate_sharing_identity(), slate_control,
                          node_configs, slate_sharer_scaling),
                      false);
        return true;
    }
    /**
     * Perform pre-slate-build initialization of BasicControl.
     *
     * @param _keychain The Keychain.
     * @param _channel_manager The channel manager, which handles all
     *                         fault-tolerant input and output data.
     * @param _adc_boards_info The parsed-out hardware configuration.
     * @param _enum_registry The set of enums to be included in telemetry
     *                       metadata.
     * @param[in, out] cycle_timer_requests The set of all CycleTimer to create.
     *
     * @return True on success.
     */
    bool BasicControl::init_pre_slate_build(
        Handle<Keychain> _keychain, Handle<FtChannelManager> _channel_manager,
        const adc_board_v &_adc_boards_info,
        Handle<EnumRegistry> _enum_registry,
        cycle_timer_name_s &cycle_timer_requests)
    {
        FswAbortIf(is_init, false);
        keychain = _keychain;
        channel_manager = _channel_manager;
        adc_boards_info = _adc_boards_info;
        enum_registry = _enum_registry;
        FswAbortIfNot(channel_manager, false);
        FswAbortIfNot(enum_registry, false);
        /*
         * Dispatch all initialization phases.
         */
        FswAbortIfNot(create_basic_initial_platform(), false);
        FswAbortIfNot(create_initial_systems(), false);
        FswAbortIfNot(create_basic_platform(), false);
        FswAbortIfNot(create_basic_shared_receiver_platform(), false);
        FswAbortIfNot(create_shared_receiver_systems(), false);
        FswAbortIfNot(create_slate_sender_creators(), false);
        FswAbortIfNot(create_basic_control_platform(), false);
        FswAbortIfNot(create_alert_system(), false);
        FswAbortIfNot(create_control_systems(cycle_timer_requests), false);
        FswAbortIfNot(create_basic_synced_command_platform(), false);
        FswAbortIfNot(populate_enums(), false);
        FswAbortIfNot(finalize_enums(), false);
        FswAbortIfNot(create_alarm_system(), false);
        /*
         * Initialize ControlCode after all control systems are initialized,
         * so that it may bind to their outputs, but prior to slate sender
         * initialization. This enables us to share outputs of ControlCode to
         * other nodes or processes.
         */
        FswAbortIfNot(create_ravenscript(cycle_timer_requests), false);
        FswAbortIfNot(create_shared_sender_systems(), false);
        FswAbortIfNot(pre_finalize_slate(), false);
        FswAbortIfNot(adc_unscale_timer.init(cycle_timer_requests), false);
        return true;
    }
    /**
     * Perform post-slate-build initialization of BasicControl.
     *
     * @param sudo_slate_control The super slate builder for the control slate.
     * @param cycle_timers The map of all CycleTimers that have been created.
     *
     * @return True on success.
     */
    bool BasicControl::init_post_slate_build(SlateBuilder sudo_slate_control,
                                             const cycle_timer_m &cycle_timers)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(adc_unscale_timer.finalize(cycle_timers), false);
        FswAbortIfNot(post_finalize_slate(sudo_slate_control), false);
        FswAbortIfNot(finalize_alarm_system(), false);
        FswAbortIfNot(create_legacy_systems(), false);
        FswAbortIfNot(build_state_machine(), false);
        FswAbortIfNot(finalize_legacy_systems(sudo_slate_control), false);
        FswAbortIfNot(finalize_basic_command_platform(sudo_slate_control),
                      false);
        FswAbortIfNot(
            finalize_control_systems(sudo_slate_control, cycle_timers), false);
        FswAbortIfNot(basic_runtime_checks(), false);
        is_init = true;
        return true;
    }
    /**
     * Get scaling info for components created by this class. The returned
     * scaling information should use paths that are relative to the absolute
     * root of Slate.
     *
     * @note If your subclass overrides this method, ensure it calls this base
     *       class method, in addition to appending its own scaling!
     *
     * @param[in,out] scaling Append scaling information to this vector.
     *
     * @return True on success.
     */
    bool BasicControl::get_scaling(scale_info_v &scaling) const
    {
        /*
         * Append scaling information from SlateSharerManager.
         */
        FswAbortIfNot(slate_sharer_manager, false);
        FswAbortIfNot(slate_sharer_manager->validate_initialized(), false);
        scaling.insert(scaling.end(), slate_sharer_scaling.begin(),
                       slate_sharer_scaling.end());
        return true;
    }
    /**
     * Start a new cycle by updating the control time.
     *
     * @param control_time The new control time.
     */
    void BasicControl::start_cycle(const nano_t control_time)
    {
        clock.override_control_time(control_time);
        /*
         * WARNING: DO NOT ADD ANYTHING ELSE TO THIS METHOD! This method is
         *          called at runtime, but is not marked RUNTIME. This is a
         *          special case that should be reserved only for setting the
         *          clock.
         */
    }
    /**
     * Dispatch the synchronized portion of the control cycle.
     *
     * @note This function has no return value to help avoid
     * accidentally FswAborting in the middle - we should not let
     * failures from a single task cancel all of vehicle control.
     */
    void BasicControl::dispatch_synced() RUNTIME
    {
        /*
         * Handle incoming Slate sharing messages.
         */
        slate_sharer_manager->dispatch_pre_control();
        /*
         * Execute vehicle-specific synchronized code.
         */
        execute_synced();
        /*
         * Reset counters if requested.
         */
        if (slate[reset_counters_tok])
        {
            reset_counters();
        }
        /*
         * Unscale the control outputs, which were generated during this cycle.
         */
        adc_unscale_timer->start();
        adc_unscaler_control->write_unscaled();
        adc_unscale_timer->stop();
        /*
         * Send vehicle-specific Slate sharing messages.
         */
        execute_send_slate_sharing();
        /*
         * Powersave any waiting valves. This must occur after control output
         * unscaling.
         */
        adc_powersaver->dispatch();
        /*
         * Send Slate sharing messages.
         */
        slate_sharer_manager->dispatch_post_control();
        /*
         * Perform time upkeep for the telemetry system. Must be called before
         * the corresponding runtime logic.
         */
        telem_relay_control->dispatch(clock.control_time());
    }
    /**
     * Send vehicle-specific Slate sharing messages.
     */
    void BasicControl::execute_send_slate_sharing() RUNTIME
    {
        FswIfNot(is_init);
    }
    /**
     * Return the identity that should be used when claiming Slate sharers in
     * SlateSharerManager. Specifically, this identity (role-inst) will be used
     * to generate the trio of hostnames that this process is running on.
     *
     * In nearly every case, those hostnames can be derived directly from the
     * process's node identity (e.g., "satfc1a", "satfc1b", and "satfc1c" for
     * "satfc1"), in which case this default implementation is sufficient.
     * However, co-located triple-string control processes will need to override
     * this method to provide the identity of their "host node".
     *
     * @return The identity to use when claiming Slate sharers.
     */
    ControlNodeIdentity BasicControl::get_slate_sharing_identity() const
    {
        return ident;
    }
    /**
     * Create the initial platform. This should include all feasible
     * global configuration information.
     *
     * @return True on success.
     */
    bool BasicControl::create_basic_initial_platform()
    {
        FswAbortIf(is_init, false);
        /*
         * Get the list of state machine commands.
         */
        FswAbortIfNot(cmd_table.init(configs), false);
        /*
         * Create our slates.
         */
        slate_shared.a = slate_control.sub_slate(ident.role_inst + 'a');
        slate_shared.b = slate_control.sub_slate(ident.role_inst + 'b');
        slate_shared.c = slate_control.sub_slate(ident.role_inst + 'c');
        /**
         * Trigger element to reset counters. It is in the cyclic shard so it
         * is automatically reset.
         */
        FswAbortIfNot(slate_control.create("reset_counters", false,
                                           shard_cyclic, slate_read_write,
                                           reset_counters_tok),
                      false);
        return true;
    }
    /**
     * Create the basic platform. This includes ADC power saver.
     *
     * @return True on success.
     */
    bool BasicControl::create_basic_platform()
    {
        FswAbortIf(is_init, false);
        /*
         * Create the raw slates.
         */
        SlateBuilder slate_control_raw = slate_control.sub_slate("raw");
        FswAbortIfNot(adc_powersaver.assume_ownership(new AdcPowersaver(clock)),
                      false);
        FswAbortIfNot(
            adc_powersaver->init(slate_control_raw,
                                 slate_control.sub_slate("adc_powersaver"),
                                 control_period, adc_boards_info, configs),
            false);
        return true;
    }
    /**
     * Create the receiver side of the shared platform. This includes
     * peer-to-peer data sharing and input handling.
     *
     * @return True on success.
     */
    bool BasicControl::create_basic_shared_receiver_platform()
    {
        FswAbortIf(is_init, false);
        /*
         * Sharing messages are processed on the same cycle on which they are
         * generated.
         */
        const nano_t cycle_delay = 0;
        /*
         * Create a slate receiver for each peer.
         */
        FswAbortIfNot(create_slate_receiver(
                          slate_shared.a,
                          sharer_get_default_local_config_list(),
                          ident.role_inst + 'a', cycle_delay, slate_receiver_a),
                      false);
        FswAbortIfNot(create_slate_receiver(
                          slate_shared.b,
                          sharer_get_default_local_config_list(),
                          ident.role_inst + 'b', cycle_delay, slate_receiver_b),
                      false);
        FswAbortIfNot(create_slate_receiver(
                          slate_shared.c,
                          sharer_get_default_local_config_list(),
                          ident.role_inst + 'c', cycle_delay, slate_receiver_c),
                      false);
        /*
         * Scale sensors for each peer data set.
         */
        FswAbortIfNot(create_adc_scaler(slate_shared.a.sub_slate("raw"),
                                        slate_shared.a, slate_control,
                                        bank_select_always,
                                        adc_scaler_shared_a),
                      false);
        FswAbortIfNot(create_adc_scaler(slate_shared.b.sub_slate("raw"),
                                        slate_shared.b, slate_control,
                                        bank_select_always,
                                        adc_scaler_shared_b),
                      false);
        FswAbortIfNot(create_adc_scaler(slate_shared.c.sub_slate("raw"),
                                        slate_shared.c, slate_control,
                                        bank_select_always,
                                        adc_scaler_shared_c),
                      false);
        return true;
    }
    /**
     * Create the control platform. This includes the input data combiner and
     * early state machine bring-up.
     *
     * @return True on success.
     */
    bool BasicControl::create_basic_control_platform()
    {
        FswAbortIf(is_init, false);
        /*
         * Combine all peer shared data.
         */
        slate_combiner_in_v combiner_in;
        combiner_in.push_back(slate_combiner_in_t(slate_shared.a, "a"));
        combiner_in.push_back(slate_combiner_in_t(slate_shared.b, "b"));
        combiner_in.push_back(slate_combiner_in_t(slate_shared.c, "c"));
        FswAbortIfNot(
            slate_combiner_control.assume_ownership(new SlateCombiner), false);
        FswAbortIfNot(slate_combiner_control->init(
                          configs, "sharer.sequence_number", combiner_in,
                          slate_control,
                          sharer_get_default_local_config_list()),
                      false);
        /*
         * Scale up combined sensors.
         */
        FswAbortIfNot(create_adc_scaler(slate_control.sub_slate("raw"),
                                        slate_control, slate_control,
                                        bank_select_always, adc_scaler_control),
                      false);
        /*
         * Scale down controlled outputs to the raw space.
         */
        FswAbortIfNot(create_adc_unscaler(slate_control,
                                          slate_control.sub_slate("raw"),
                                          adc_unscaler_control),
                      false);
        /*
         * Create the state machine.
         */
        FswAbortIfNot(state_machine.assume_ownership(
                          new SyncStateMachine(cmd_table, control_period)),
                      false);
        FswAbortIfNot(
            state_machine->init_storage(configs, slate_control, ident.role),
            false);
        /*
         * Initialize ftrace slate tokens.
         */
        FswAbortIfNot(FtraceTrap::init_device(slate_control), false);
        for (const auto &string_letter : {"a", "b", "c"})
        {
            FswAbortIfNot(FtraceTrap::init_device(
                              slate_control, ident.role_inst + string_letter),
                          false);
        }
        /*
         * Instantiate the enumerated bitmaps. This part creates all the
         * enum Slate elements.
         */
        FswAbortIfNot(
            enumerated_bitmaps.assume_ownership(new EnumeratedBitmapManager),
            false);
        FswAbortIfNot(enumerated_bitmaps->init_enums(slate_control, configs,
                                                     *enum_registry),
                      false);
        /*
         * Create the dummy autosequence_begin and autosequence_end devices,
         * which are conventionally used to mark the beginning and ending of
         * event sequences.
         */
        WriteToken<bool> autosequence_begin;
        FswAbortIfNot(slate_control.create("autosequence_begin", false,
                                           shard_cyclic, slate_read_write,
                                           autosequence_begin),
                      false);
        WriteToken<bool> autosequence_end;
        FswAbortIfNot(slate_control.create("autosequence_end", false,
                                           shard_cyclic, slate_read_write,
                                           autosequence_end),
                      false);
        /*
         * Create the control elements for the telemetry system.
         */
        FswAbortIfNot(
            telem_relay_control.assume_ownership(new TelemetryRelayControl()),
            false);
        FswAbortIfNot(telem_relay_control->init("telem", slate_control, configs,
                                                telem_config_key),
                      false);
        return true;
    }
    /**
     * Create the synced command platform.
     *
     * @return True on success.
     */
    bool BasicControl::create_basic_synced_command_platform()
    {
        FswAbortIf(is_init, false);
        const std::string dispatcher_name = "gnd_cmd";
        /*
         * Get the input channels.
         */
        std::vector<Handle<DgramChannel>> inputs;
        FswAbortIfNot(
            external_command_get_default_synced_inputs(channel_manager, inputs),
            false);
        /*
         * Specify a synced Slate shard in the calls below.
         */
        const slate_shard_t shard = shard_sync;
        /*
         * Create the deframer.
         */
        Handle<ExternalCommandDeframer> deframer;
        FswAbortIfNot(create_synced_command_deframer(deframer), false);
        /*
         * Create the time filter.
         */
        Handle<ExternalCommandTimeFilter> time_filter(
            new ExternalCommandTimeFilterNull());
        FswAbortIfNot(time_filter, false);
        Handle<ExternalCommandFilter> cmd_filter;
        FswAbortIfNot(create_synced_command_filter(cmd_filter), false);
        /*
         * We don't use a command armer.
         */
        Handle<ExternalCommandArmer> armer(new ExternalCommandArmerNull());
        FswAbortIfNot(armer, false);
        /*
         * Create the command handlers.
         */
        ext_cmd_handler_v handlers;
        FswAbortIfNot(create_synced_command_handlers(dispatcher_name, handlers),
                      false);
        /*
         * Create the output channels.
         */
        std::map<std::string, Handle<Channel>> outputs;
        FswAbortIfNot(external_command_get_default_synced_outputs(
                          channel_manager, outputs),
                      false);
        /*
         * Create and initialize the command dispatcher.
         */
        FswAbortIfNot(ground_cmd_dispatcher_synced.assume_ownership(
                          new ExternalCommandDispatcher(dispatcher_name, clock,
                                                        ext_cmd_params)),
                      false);
        FswAbortIfNot(ground_cmd_dispatcher_synced->init(
                          slate_control, ident.role_inst, shard, inputs,
                          deframer, time_filter, cmd_filter, armer, handlers,
                          outputs),
                      false);
        /*
         * Look up the disconnect command for the spammer below.
         */
        vehicle_cmd_t disconnect_cmd;
        FswAbortIfNot(cmd_table.lookup("disconnect", disconnect_cmd), false);
        /*
         * Create the disconnect command spammer.
         */
        FswAbortIfNot(
            disconnect_command_spammer.assume_ownership(
                new CommandSpammer(clock, state_machine, "disconnect_spammer")),
            false);
        FswAbortIfNot(
            disconnect_command_spammer->init(slate_control, control_period),
            false);
        /*
         * Attach the all disconnect signal from the watchdog to the disconnect
         * command spammer.
         */
        NodeWatchdog::all_disconnect_sig_t &all_disconnect_sig =
            ground_cmd_dispatcher_synced->get_watchdog_all_disconnect_sig();
        FswAbortIfNot(all_disconnect_sig.connect(
                          RUNTIME_BIND(make_slot(*disconnect_command_spammer,
                                                 &CommandSpammer::handle_cmd),
                                       disconnect_cmd)),
                      false);
        /*
         * Create some devices that can be used to test commanding.
         */
        slate_element_t element_id = 0;
        FswAbortIfNot(slate_control.create_element<INT64>(
                          "null_int", 0, shard, slate_read_only, element_id),
                      false);
        FswAbortIfNot(slate_control.create_element<double>(
                          "null_fp", 0.0, shard, slate_private, element_id),
                      false);
        return true;
    }
    /**
     * Add any final enumerations to the enum registry. No new enumerations for
     * telemetry and event files will be accepted after this point.
     *
     * @return True on success.
     */
    bool BasicControl::finalize_enums()
    {
        FswAbortIf(is_init, false);
        /*
         * Register any enums that can be used in telemetry or event files.
         */
        str_v state_machine_channels;
        const std::string sm_prefix = ident.role_inst + "x.sm_" + ident.role;
        state_machine_channels.push_back(sm_prefix + ".current_state");
        state_machine_channels.push_back(sm_prefix + ".old_state");
        state_machine_channels.push_back(sm_prefix + ".older_state");
        FswAbortIfNot(enum_registry->register_enum(
                          "state_machine", state_machine->get_state_sym_table(),
                          state_machine_channels),
                      false);
        return true;
    }
    /**
     * Perform all final initialization before the slate is built.
     *
     * @return True on success.
     */
    bool BasicControl::pre_finalize_slate()
    {
        FswAbortIf(is_init, false);
        /*
         * Finalize the SlateSharerManager, which will attach node I/O channels
         * and resolve any outstanding Slate element binds.
         */
        FswAbortIfNot(alert_provider, false);
        FswAbortIfNot(
            slate_sharer_manager->finalize(alert_provider, channel_manager),
            false);
        /*
         * Bind enumerated bitmaps to their output Slate elements.
         */
        FswAbortIfNot(enumerated_bitmaps->init_outputs(slate_control), false);
        /*
         * Get the runtime.
         */
        slate = slate_control.slate(slate_no_validation);
        return true;
    }
    /**
     * Perform early initialization post-slate built.
     *
     * @param sudo_slate_control The super slate builder for the control slate.
     *
     * @return True on success.
     */
    bool BasicControl::post_finalize_slate(SlateBuilder sudo_slate_control)
    {
        return true;
    }
    /**
     * Finalize the legacy systems by initializing the state machine.
     *
     * @param sudo_slate_control The super slate builder for the control slate.
     *
     * @return True on success.
     */
    bool BasicControl::finalize_legacy_systems(SlateBuilder sudo_slate_control)
    {
        /*
         * Initialize the state machine.
         */
        FswAbortIfNot(
            state_machine->init(sudo_slate_control, configs, *enum_registry),
            false);
        return true;
    }
    /**
     * Post-slate-creation, finalize setup of the commanding platform.
     *
     * @param sudo_slate_control The super slate builder for the control slate.
     *
     * @return True on success.
     */
    bool BasicControl::finalize_basic_command_platform(
        SlateBuilder sudo_slate_control)
    {
        FswAbortIf(is_init, false);
        /*
         * Initialize the Slate command interfaces.
         */
        FswAbortIfNot(slate_command_interface_control, false);
        FswAbortIfNot(slate_command_interface_control->init(
                          sudo_slate_control, ident.role_inst + "x"),
                      false);
        return true;
    }
    /**
     * Build the state machine control task set. Alarms are always first.
     *
     * @return True on success.
     */
    bool BasicControl::build_state_machine()
    {
        FswAbortIf(is_init, false);
        /*
         * Add other pre-transition tasks.
         */
        FswAbortIfNot(build_state_machine_pre_transition(), false);
        /*
         * Add alarms.
         */
        FswAbortIfNot(
            state_machine->add_ctask("multi_sensor_alarm", slate_alarms),
            false);
        /*
         * Add post-transition tasks.
         */
        FswAbortIfNot(state_machine->pre_transition_ctasks_added(), false);
        FswAbortIfNot(build_state_machine_post_transition(), false);
        return true;
    }
    /**
     * Perform sanity checks before runtime.
     *
     * @return True on success.
     */
    bool BasicControl::basic_runtime_checks()
    {
        /*
         * Ensure there are no uninitialized commands.
         */
        FswAbortIfNeq(num_uninitialized_vehicle_cmds(), 0, false);
        FswAbortIfNot(runtime_checks(), false);
        return true;
    }
    /**
     * Vehicles should use this method to initialize vehicle-specific
     * configuration data.
     *
     * @return True on success.
     */
    bool BasicControl::create_initial_systems()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to create the receiving side of systems
     * which populate the peer and remote sharing slates with data.
     *
     * @return True on success.
     */
    bool BasicControl::create_shared_receiver_systems()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to create slate senders which need to
     * create their slate elements rather than bind to them. This is called at
     * the beginning of the initialization process rather than the end where
     * the rest of the senders are initialized.
     *
     * @return True on success.
     */
    bool BasicControl::create_slate_sender_creators()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Create the default alert system. Vehicle should override this function
     * to provide their own.
     *
     * @return True on success.
     */
    bool BasicControl::create_alert_system()
    {
        FswAbortIf(is_init, false);
        FswAbortIf(alert_provider, false);
        Handle<VoidAlertProvider> alerts(new VoidAlertProvider());
        FswAbortIfNot(alerts, false);
        FswAbortIfNot(alerts->init(slate_control.sub_slate("alerts")), false);
        alert_provider = alerts;
        return true;
    }
    /**
     * Vehicles should use this method to create systems which populate
     * the control slate with data.
     *
     * @param[in, out] cycle_timer_requests The set of all CycleTimer to create.
     *
     * @return True on success.
     */
    bool BasicControl::create_control_systems(
        cycle_timer_name_s &cycle_timer_requests)
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to create the sending side of systems
     * which populate the peer and remote sharing slates with data.
     *
     * @return True on success.
     */
    bool BasicControl::create_shared_sender_systems()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to create a synced command deframer.
     *
     * @param[out] deframer The synced command deframer.
     *
     * @return True on success.
     */
    bool BasicControl::create_synced_command_deframer(
        Handle<ExternalCommandDeframer> &deframer)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(
            deframer.assume_ownership(new ExternalCommandDeframerNull()),
            false);
        return true;
    }
    /**
     * Vehicles should use this method to create a synced command filter.
     *
     * @param[out] cmd_filter The synced command filter.
     *
     * @return True on success.
     */
    bool BasicControl::create_synced_command_filter(
        Handle<ExternalCommandFilter> &cmd_filter)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(
            cmd_filter.assume_ownership(new ExternalCommandFilterNull()),
            false);
        return true;
    }
    /**
     * Vehicles should use this method to create command handlers
     * to be used by the ExternalCommandDispatcher.
     *
     * @param dispatcher_name Name of the command dispatcher (used as a base
     * for command handler names).
     * @param[out] handlers Command handlers.
     *
     * @return True on success.
     */
    bool BasicControl::create_synced_command_handlers(
        const std::string &dispatcher_name, ext_cmd_handler_v &handlers)
    {
        FswAbortIf(is_init, false);
        /*
         * Create the device command handler.
         */
        Handle<ExternalCommandSlateHandler> device_handler(
            new ExternalCommandSlateHandler());
        FswAbortIfNot(device_handler, false);
        FswAbortIfNot(slate_command_interface_control.assume_ownership(
                          new SlateCommandInterface()),
                      false);
        FswAbortIfNot(device_handler->init(slate_command_interface_control),
                      false);
        handlers.push_back(device_handler);
        /*
         * Create the StateMachine command handler.
         */
        Handle<ExternalCommandStateMachineHandler> sm_handler(
            new ExternalCommandStateMachineHandler(
                dispatcher_name + "_sm_handler", clock));
        FswAbortIfNot(sm_handler, false);
        FswAbortIfNot(
            sm_handler->init(slate_control, shard_sync, state_machine), false);
        handlers.push_back(sm_handler);
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
    bool
    BasicControl::finalize_control_systems(SlateBuilder sudo_slate_control,
                                           const cycle_timer_m &cycle_timers)
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to create systems which make use
     * of the deprecated devices subsystem.
     *
     * @return True on success.
     */
    bool BasicControl::create_legacy_systems()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to add enumerations to the enum
     * registry.
     *
     * @return True on success.
     */
    bool BasicControl::populate_enums()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to perform runtime checks.
     *
     * @return True on success.
     */
    bool BasicControl::runtime_checks()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to add pre-transition control
     * tasks to the state machine.
     *
     * @return True on success.
     */
    bool BasicControl::build_state_machine_pre_transition()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to add post-transition control
     * tasks to the state machine.
     *
     * @return True on success.
     */
    bool BasicControl::build_state_machine_post_transition()
    {
        FswAbortIf(is_init, false);
        return true;
    }
    /**
     * Vehicles should use this method to initialize Ravenscript, if desired.
     *
     * By default, BasicControl does nothing.
     *
     * @param[in, out] cycle_timer_requests The set of all CycleTimer to create.
     *
     * @return True on success.
     */
    bool
    BasicControl::create_ravenscript(cycle_timer_name_s &cycle_timer_requests)
    {
        return true;
    }
    /**
     * Creates the slate alarm system.
     *
     * @return True on success.
     */
    bool BasicControl::create_alarm_system()
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(alert_provider, false);
        FswAbortIfNot(alarm_sequence_handler.init_storage(
                          configs, slate_control, "alarm_event_sequences"),
                      false);
        FswAbortIf(slate_alarms, false);
        FswAbortIfNot(slate_alarms.assume_ownership(
                          new SlateAlarmTask("multi_sensor_alarm")),
                      false);
        FswAbortIfNot(slate_alarms->init(
                          control_period, slate_control, slate_control,
                          cmd_table, *alert_provider, configs,
                          true /* can_be_stopped */, "multi_sensor_alarms"),
                      false);
        /*
         * All other enums have been registered by the time create_alarm_system
         * executes so it is safe to immediately populate enums for Multi Sensor
         * Alarms here. This allows Multi Sensor Alarm threshold devices to be
         * paired with the matching enum of their sensor devices.
         */
        FswAbortIfNot(slate_alarms->populate_enums(*enum_registry), false);
        /*
         * Set up a token to store alarm_inhibit.
         */
        FswAbortIfNot(slate_control.create("alarm_inhibit", 0, shard_sync,
                                           slate_private, alarm_inhibit_tok),
                      false);
        /*
         * Handle alarm signals.
         */
        FswAbortIfNot(
            slate_alarms->alarm_sig.connect(RUNTIME_BIND(
                make_slot(*this, &BasicControl::handle_alarm), state_machine)),
            false);
        return true;
    }
    /**
     * Finalize the slate alarm system.
     *
     * @return True on success.
     */
    bool BasicControl::finalize_alarm_system()
    {
        FswAbortIf(is_init, false);
        /*
         * Initialize the AlarmEventSequenceHandler. This must be done before
         * any alarms are initialized.
         */
        FswAbortIfNot(
            alarm_sequence_handler.init(slate_control, configs, cmd_table,
                                        "alarm_event_sequences", *enum_registry,
                                        ident.role_inst + 'x'),
            false);
        const std::vector<Handle<SlateAlarm>> &alarms =
            slate_alarms->get_alarms();
        FswAbortIfNot(alarm_sequence_handler.validate_alarms(alarms), false);
        return true;
    }
    /**
     * Create a SlateSharerSender.
     *
     * @param source The slate to pull data from.
     * @param config_list The list of config files to use.
     * @param sender_name Name used for internal slate for this sender.
     * @param output_channel Write to this output channel with this name.
     * @param is_triple_string True if the output is identical for all strings.
     * @param create_or_bind Whether to create the shared devices in the slate
     *                       or bind to existing ones.
     * @param[out] sender Returns the constructed sender.
     *
     * @return True on success.
     */
    bool BasicControl::create_slate_sender(
        SlateBuilder source, const sharer_config_v &config_list,
        const std::string &sender_name, const std::string &output_channel,
        const bool is_triple_string,
        const sharer_create_or_bind_t create_or_bind,
        Handle<SlateSharerSender> &sender)
    {
        FswAbortIf(is_init, false);
        FswAbortIf(sender, false);
        Handle<Channel> out;
        if (is_triple_string)
        {
            FswAbortIfNot(
                channel_manager->get_triple_string_output(output_channel, out),
                false);
        }
        else
        {
            FswAbortIfNot(channel_manager->get_output(output_channel, out),
                          false);
        }
        FswAbortIfNot(sender.assume_ownership(new SlateSharerSender(clock)),
                      false);
        FswAbortIfNot(sender->init(sender_name, source, shard_sync, configs,
                                   config_list, out, create_or_bind),
                      false);
        return true;
    }
    /**
     * Create a SlateSharerReceiver.
     *
     * @param destination Drop data into this slate.
     * @param builder The slate to pull data from.
     * @param config_list The list of config files to use.
     * @param input_channel Read from the input channel with this name.
     * @param cycle_delay Expected cycle delay from sending to receiving, used
     * to validate incoming messages.
     * @param[out] receiver Returns the constructed receiver.
     *
     * @return True on success.
     */
    bool BasicControl::create_slate_receiver(
        SlateBuilder destination, const sharer_config_v &config_list,
        const std::string &input_channel, const nano_t cycle_delay,
        Handle<SlateSharerReceiver> &receiver)
    {
        FswAbortIf(is_init, false);
        FswAbortIf(receiver, false);
        Handle<DgramChannel> in;
        FswAbortIfNot(channel_manager->get_dgram_input(input_channel, in),
                      false);
        FswAbortIfNot(receiver.assume_ownership(
                          new SlateSharerReceiver(clock, cycle_delay)),
                      false);
        /*
         * SlateSharerReceiver outputs should ideally be in the cyclic shard :
         * - They are only valid for the current cycle.
         * - It minimizes the size of the sync shard.
         * Code consuming string specific data should always verify that the
         * connection with the string is active.
         *
         * However, until we are certain we don't have existing code that access
         * this data without checking the connection status, for backward
         * compatibility and to minimize chances of regression, we create them
         * in the sync shard.
         */
        FswAbortIfNot(receiver->init("sharer", destination, shard_sync, configs,
                                     config_list, in, sharer_create),
                      false);
        return true;
    }
    /**
     * Create an AdcScaler.
     *
     * @param raw Read raw data from here.
     * @param scaled Write scaled data to here.
     * @param median Slate to check if a scaled-up value is in the median slate.
     * @param ad_scaling_set Specify the set of AD sensors to be scaled.
     *
     * @param[out] scaler Returns the constructed scaler.
     *
     * @return True on success.
     */
    bool BasicControl::create_adc_scaler(
        SlateBuilder raw, SlateBuilder scaled, SlateBuilder median,
        const bank_select_type_t ad_scaling_set, Handle<AdcScaler> &scaler)
    {
        FswAbortIf(scaler, false);
        bank_select_t bank_select = {};
        FswAbortIfNot(AdcScaler::select_all_banks(bank_select), false);
        bank_select[ad_dev_t] = ad_scaling_set;
        FswAbortIfNot(scaler.assume_ownership(new AdcScaler), false);
        FswAbortIfNot(scaler->init(raw, scaled, median, adc_boards_info,
                                   bank_select, adc_scaler_exclude_channels),
                      false);
        return true;
    }
    /**
     * Create an AdcUnscaler.
     *
     * @param scaled Read scaled data from here.
     * @param raw Write raw data to here.
     * @param[out] unscaler Returns the constructed unscaler.
     *
     * @return True on success.
     */
    bool BasicControl::create_adc_unscaler(SlateBuilder scaled,
                                           SlateBuilder raw,
                                           Handle<AdcUnscaler> &unscaler)
    {
        FswAbortIf(unscaler, false);
        FswAbortIfNot(unscaler.assume_ownership(new AdcUnscaler), false);
        FswAbortIfNot(unscaler->init(scaled, raw, shard_sync, adc_boards_info),
                      false);
        return true;
    }
    /**
     * Handles an alarm generated from the MultiSensorAlarms.
     *
     * @param alarm A handle to the alarm that was triggered.
     *
     * @param sm The StateMachine to which to send the command associated with
     *           the alarm.
     *
     * @return True to acknowledge the alarm. False to indicate that
     *         the MultiSensorAlarm task needs to call back this function
     *         again.
     */
    bool BasicControl::handle_alarm(RUNTIME SlateAlarm &alarm,
                                    Handle<StateMachine> sm) RUNTIME
    {
        FswAbortIfNot(is_init, false);
        /*
         * Ignore all alarms that have a level less than or equal to
         * the current alarm inhibit value.
         */
        const int alarm_inhibit = slate.load(alarm_inhibit_tok);
        if (alarm.level <= alarm_inhibit)
        {
            return true;
        }
        alarm.display_alarm_message();
        /*
         * If the alarm wants us to run an event sequence in response
         * instead of requesting a state transition, do that. For all
         * other commands, request the state transition.
         */
        if (alarm_sequence_handler.alarm_runs_event_sequence(alarm))
        {
            FswAbortIfNot(alarm_sequence_handler.run_sequence(alarm), false);
        }
        else
        {
            vc_cmd_response_t response;
            nano_t retry_at;
            FswAbortIfNot(sm->handle_cmd(clock.control_time(), alarm.command,
                                         response, retry_at),
                          false);
            if (response == vc_cmd_defer)
            {
                /*
                 * Returning false indicates that we cannot accept the
                 * command at this time.
                 */
                return false;
            }
        }
        /*
         * The command was handled.
         */
        return true;
    }
    /**
     * Reset the counters of all components.
     *
     * @return True on success.
     */
    void BasicControl::reset_counters() RUNTIME {}
} /* end namespace Drone */