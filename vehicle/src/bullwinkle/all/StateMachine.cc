/**
 * @author Chris Sloan
 * @date   11/11/04
 */
#include "src/bullwinkle/all/StateMachine.h"
#include "src/bullwinkle/all/Crc.h"
#include "src/bullwinkle/all/Periodic.h"
#include "src/bullwinkle/all/StdioFile.h"
#include "src/bullwinkle/all/core/stl_util.h"
#include "src/bullwinkle/all/ctask_state_t.enum.h"
#include "src/bullwinkle/all/file/file_parse.h"
#include <algorithm>
#include <map>
namespace Drone
{
    /**
     * Constant used to indicate that a command has no defined
     * action in a particular state. Maximum value.
     */
    const uint StateMachine::undefined_state = UINT_MAX;
    /**
     * Constant used to indicate that a command is to be ignored
     * in a particular state. Maximum value minus 1.
     */
    const uint StateMachine::ignore_state = UINT_MAX - 1;
    /**
     * Constant used as the default verbosity of the messages printed by a
     * StateMachine. Can be changed in the constructor.
     */
    const int StateMachine::default_verbosity = 1;
    /**
     * Constructor.
     *
     * @param _cmd_table Reference to the CommandTable. Used to look
     * up vehicle commands.
     * @param _transition_period The period at which to check for
     * state transitions.
     * @param _local_verbosity   Affects what dbverbose() level is
     * required before debug messages are printed.
     * @param _initial_control_time Time to use to initialize the initial event
     * sequence.
     */
    StateMachine::StateMachine(const CommandTable &_cmd_table,
                               const nano_t _transition_period,
                               const int _local_verbosity,
                               const nano_t _initial_control_time)
        : EventSource("StateMachine"), is_init(false), is_init_storage(false),
          initial_control_time(_initial_control_time), slate(),
          state_machine_name(), role(), cmd_table(_cmd_table), state_v(),
          state_sym(), ctask_sym(), task(), state(), timeout_cmd(),
          pending_cmd_index_tok(), transition_period(_transition_period),
          nominal_transition_time_tok(), pre_transition_tasks_added(false),
          num_pre_transition_ctasks(0), local_verbosity(_local_verbosity),
          event_sequence_time_tok(), cur_state_tok(), old_state_tok(),
          older_state_tok()
    {}
    /**
     * Destructor.
     */
    StateMachine::~StateMachine() {}
    /**
     * Read the state file, pre-allocate and initialize storage of objects.
     *
     * @param configs Config file finder.
     * @param builder The slate where to store this state machine's elements.
     * The subtree path of this slate is also used when resolving device names
     * in the enum registry, since the enum registry is always fully qualified
     * and devices in event sequence files are not. Devices listed in an event
     * sequence file have the path of this Slate prefixed to their name to
     * obtain a full Slate path.
     * @param _role The computer's role (ffc, sfc, etc.) which is used to find
     * configuration files and to decide which state machine to use.
     *
     * @return True on success.
     */
    bool StateMachine::init_storage(const Configs &configs,
                                    SlateBuilder builder,
                                    const std::string &_role)
    {
        /*
         * Ensure that it is only initialized once.
         */
        FswAbortIf(is_init, false);
        FswAbortIf(is_init_storage, false);
        /*
         * Save role and slate sub-tree path to be able to resolve device names
         * during construction of EventSequences.
         */
        role = _role;
        /*
         * Set the default name of the state machine to the role.
         */
        state_machine_name = role;
        /*
         * Make the slate tokens.
         */
        slate = builder.slate(slate_no_validation);
        SlateBuilder sub = builder.sub_slate("sm_" + role);
        /*
         * These slate elements are used for telemetry. event_sequence_time is
         * also used to ensure that the three strings of computers are in sync,
         * so we place this one into the sync shard.
         */
        FswAbortIfNot(sub.create("event_sequence_time", 0.0, shard_sync,
                                 slate_read_only, event_sequence_time_tok),
                      false);
        FswAbortIfNot(sub.create("old_state", -1, shard_sync, old_state_tok),
                      false);
        FswAbortIfNot(
            sub.create("older_state", -1, shard_sync, older_state_tok), false);
        /*
         * State machine runtime state. It needs to be in the shard_sync slate
         * for HotSync.
         */
        FswAbortIfNot(sub.create("current_state", 0, shard_sync,
                                 slate_read_only, cur_state_tok),
                      false);
        FswAbortIfNot(sub.create("pending_cmd_index", noop_cmd_index,
                                 shard_sync, pending_cmd_index_tok),
                      false);
        FswAbortIfNot(sub.create("nominal_transition_time", nano_t_max,
                                 shard_sync, nominal_transition_time_tok),
                      false);
        /*
         * Let sub-class initialize storage.
         */
        FswAbortIfNot(init_storage_state_machine(builder), false);
        /*
         * Initialize symbol table. We need to do this during init_storage()
         * in order to know which state files to parse and initialize storage
         * of EventSequence objects.
         */
        /*
         * Lookup the special commands.
         */
        FswAbortIfNot(cmd_table.lookup("timeout", timeout_cmd), false);
        /*
         * Read the master configuration file to get the list of
         * states.
         */
        std::string master_file;
        FswAbortIfNot(configs.config_file(role + ".master", master_file),
                      false);
        FswAbortIfNot(read_word_file(master_file, state_v), false);
        FswAbortIfEqInt(state_v.size(), 0, false);
        /*
         * Setup the state symbol table so that we can convert to and from state
         * names and state numbers.
         */
        for (size_t i = 0; i < state_v.size(); i++)
        {
            FswAbortIfNot(state_sym.add(state_v[i], i), false);
        }
        /*
         * Perform a simple check for duplicate state names in the master file.
         */
        for (size_t i = 0; i + 1 < state_v.size(); i++)
        {
            for (size_t j = i + 1; j < state_v.size(); j++)
            {
                if (FswIf(state_v[i] == state_v[j]))
                {
                    FswPrefix();
                    dbnprintf(400,
                              ": Detected duplicate state: %s in master "
                              "file %s\n",
                              lookup_state(i).c_str(), master_file.c_str());
                    return false;
                }
            }
        }
        /*
         * Add an additional symbol for "ignore."  If there is no
         * defined action for a command in the current state, then a
         * warning will be issued.  The "ignore" pseudo-state can be
         * used to suppress this warning for commands which are
         * expected but for which we do not wish to transition states.
         */
        FswAbortIfNot(state_sym.add("ignore", ignore_state), false);
        /*
         * Build and initialize storage for all ControlStates and EventSequences
         * needed by reading descriptions of each state from the config files.
         */
        state.resize(state_v.size());
        for (size_t i = 0; i < state_v.size(); i++)
        {
            /*
             * Create ControlState.
             */
            state[i].assume_ownership(new ControlState());
            FswAbortIfNot(state[i], false);
            /*
             * Parse state file to initialize all EventSequences.
             */
            FswMsgAbortIfNot(initialize_state_from_file(configs, builder, i),
                             false, 100, "Error reading file for state : %s",
                             state_v[i].c_str());
        }
        is_init_storage = true;
        return true;
    }
    /**
     * Read the configuration files, set up the state machine and transition the
     * state machine to its initial state.
     *
     * @param builder The SlateBuilder
     * @param configs Config file finder.
     * @param enum_registry Enumerations used for translating names to values in
     * event files.
     *
     * @return True on success.
     */
    bool StateMachine::init(SlateBuilder builder, const Configs &configs,
                            const EnumRegistry &enum_registry)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(is_init_storage, false);
        FswAbortIfNot(enum_registry.is_finalized(), false);
        str_str_s_m multi_seq_devices;
        FswAbortIfNot(
            parse_multi_seq_devices(builder, configs, role, multi_seq_devices),
            false);
        /*
         * Loop on all states and parse corresponding files.
         */
        for (size_t i = 0; i < state.size(); i++)
        {
            /*
             * Finish initialization of ControlState.
             */
            FswAbortIfNot(state[i]->init(cmd_table.num_commands(), task.size(),
                                         undefined_state, state_v[i]),
                          false);
            /*
             * Parse EventSequence files.
             */
            ControlState::sequence_map_t &seqs = state[i]->seqs;
            for (ControlState::sequence_map_t::iterator seq_iter = seqs.begin();
                 seq_iter != seqs.end(); seq_iter++)
            {
                EventSequence &seq = *seq_iter->second;
                std::string seq_file;
                FswAbortIfNot(configs.config_file(seq_iter->first, seq_file),
                              false);
                FswAbortIfNot(
                    seq.read(seq_file, nano_t_max, builder, enum_registry),
                    false);
            }
            /*
             * Finish parsing state file for tasks and cmds.
             */
            FswMsgAbortIfNot(
                finalize_state_from_file(configs, i, multi_seq_devices), false,
                100,
                "State machine '%s': "
                "error reading file for state '%s'.",
                role.c_str(), state_v[i].c_str());
            /*
             * Validate the event sequences.
             */
            FswAbortIfNot(state[i]->validate_event_sequences(), false);
        }
        /*
         * Validate all states and make sure that they all hash to a
         * unique CRC.
         */
        std::map<uint, std::string> crc_state_map;
        for (size_t i = 0; i < state.size(); i++)
        {
            FswAbortIfNot(state[i], false);
            if (!validate_state(*(state[i]), i))
            {
                FswAbortOutsideRangeUint(i, 0, state_v.size(), false);
                FswPrefix();
                dbnprintf(100, ": Failed to parse state '%s'\n",
                          lookup_state(i).c_str());
                return false;
            }
            const std::string this_state = lookup_state(i);
            const uint this_crc = hash_state(this_state);
            std::string other_state;
            if (map_find(crc_state_map, this_crc, other_state))
            {
                FswPrefix();
                dbnprintf(200,
                          ": States '%s' and '%s' hash to the same value, "
                          "please rename one of them.\n",
                          this_state.c_str(), other_state.c_str());
                return false;
            }
            crc_state_map[this_crc] = this_state;
        }
        FswAbortIfNot(init_finalize(), false);
        FswMsgAbortIfNot(set_initial_state(), false, 100,
                         "Failed to initialize state machine '%s'.",
                         role.c_str());
        is_init = true;
        return true;
    }
    /**
     * Get the current state name.
     *
     * @note ControlTasks and such are not supposed to use this function. The
     * goal is to make them state agnostic. This leaves the configuration files
     * free to fully specify behavior instead of having behaviour hard coded
     * based on state numbers or names.
     *
     * @return Return the name of the current state.
     */
    std::string StateMachine::get_state() const
    {
        return lookup_state(slate[cur_state_tok]);
    }
    /**
     * Produce a hash of a state name that's guaranteed to be unique within a
     * given state machine.
     *
     * @param state State name to hash.
     *
     * @return Hash of the state name.
     */
    uint StateMachine::hash_state(const std::string &state)
    {
        return Crc::crc32().crc(state);
    }
    /**
     * Get the current state number.
     *
     * @note ControlTasks and such are not supposed to use this function. The
     * goal is to make them state agnostic. This leaves the configuration files
     * free to fully specify behavior instead of having behaviour hard coded
     * based on state numbers or names.
     *
     * @return Return the number of the current state.
     */
    uint StateMachine::get_state_num() const { return slate[cur_state_tok]; }
    /**
     *  Adds a control task to the state machine.
     *
     *  @param ctask_name The name of the control task to add.
     *  @param ctask A handle to the control task to add.
     *
     *  @return True on success.
     */
    bool StateMachine::add_ctask(const std::string ctask_name,
                                 Handle<ControlTask> ctask)
    {
        /*
         * Check the control task for NULL.
         */
        FswAbortIfNot(ctask, false);
        /*
         * Add the task to the SymbolTable.
         */
        uint ctask_num = task.size();
        FswAbortIfNot(ctask_sym.add(ctask_name, ctask_num), false);
        /*
         * Add the task to the control task vector.
         */
        task.push_back(ctask);
        return true;
    }
    /**
     * Indicates that all pre-transition tasks have been added to the
     * StateMachine.
     *
     * All ControlTasks added to the StateMachine before this function
     * is called will be executed before the StateMachine performs the state
     * transition check. All ControlTasks added after this call will be
     * executed after the state transition check.
     *
     * @see add_ctask
     *
     * @return True on success.
     */
    bool StateMachine::pre_transition_ctasks_added()
    {
        FswAbortIf(is_init, false);
        FswAbortIf(pre_transition_tasks_added, false);
        /*
         * Record the number of tasks currently added to the task vector.
         */
        num_pre_transition_ctasks = task.size();
        pre_transition_tasks_added = true;
        return true;
    }
    /**
     * Sets the name of the state machine that is used for console prints.
     *
     * @param name The name.
     *
     * @return True on success.
     */
    bool StateMachine::set_state_machine_name(const std::string &name)
    {
        state_machine_name = name;
        return true;
    }
    /**
     * Get the command table.
     *
     * @return The command table.
     */
    const CommandTable &StateMachine::get_cmd_table() const
    {
        return cmd_table;
    }
    /**
     * Get the state symbol table.
     *
     * @return The state symbol table.
     */
    const SymbolTable &StateMachine::get_state_sym_table() const
    {
        return state_sym;
    }
    /**
     * Determine if the supplied command is allowed in the current state.
     *
     * @details A command is allowed if it is a noop, if the command will be
     * explicitly ignored by the StateMachine, or if the command leads to a
     * valid state. Even if the command is allowed, the command may still be
     * deferred when handled due to a currently pending command.
     *
     * @note ControlTasks and such are not supposed to use this function. The
     * goal is to make them state agnostic. This leaves the configuration files
     * free to fully specify behavior instead of having behaviour hard coded
     * based on state numbers or names.
     *
     * @param cmd The command to check.
     *
     * @return True if the command is allowed.
     */
    bool StateMachine::is_cmd_allowed(vehicle_cmd_t cmd) const
    {
        if (cmd.is_noop())
        {
            return true;
        }
        FswAbortOutsideRange((uint)slate[cur_state_tok], 0U, state.size(),
                             false);
        Handle<RUNTIME ControlState> current_state =
            state[slate[cur_state_tok]];
        FswAbortIfNot(current_state, false);
        /*
         * The ControlState stores a vector of state numbers, indexed by
         * commands.
         */
        const uint cmd_index = cmd.get_index();
        FswAbortOutsideRange(cmd_index, 0U, current_state->cmds.size(), false);
        uint next_state = current_state->cmds[cmd_index];
        FswAbortIfOpInt(static_cast<size_t>(next_state), >=, state.size(),
                        false);
        if (next_state == ignore_state)
        {
            /*
             * Even though the command is ignored, the StateMachine should
             * successfully handle it, so the command is allowed.
             */
            return true;
        }
        else if (next_state == undefined_state)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    /**
     * Request a state change based on the supplied command.
     *
     * @details
     * Unlike previous versions of StateMachine, the transition will
     * not occur immediately, but at the next scheduled transition
     * point.
     *
     * The supplied commands can be accepted, deferred, or rejected.
     *
     * Accepted commands will be carried out at the next scheduled
     * transition point.
     *
     * Commands can be deferred if the state machine cannot carry out
     * the command at this time. The main reason for deferring a
     * command is if a command is currently pending, but more reasons
     * may be added in the future.
     *
     * A command can be rejected if it fails a validity check or an
     * error occurs validating it. It may also be rejected if the
     * command is not valid in the current mode. This occurs if there
     * is no defined action for the command in the current mode.
     * (Note that a command with an action of "ignore" is not rejected
     * as it has a defined action.)
     *
     * If the command is deferred, the caller should usually call
     * again later with the same command. For example, if a network
     * disconnect message is deferred, the caller should send it again
     * later. As another example however, if a MultiSensorAlarm is
     * triggered, the command should not be resumbitted unless the
     * condition still exists after the pending mode change completes.
     * This is required since the mode change may disable the alarm or
     * alter its conditions.
     *
     * If a command is rejected, the caller should discard the
     * command.
     *
     * @param control_time The current control time.
     * @param cmd The command to handle.
     * @param response Returns the response to the command.
     * @param retry_at Returns the time to next retry at if @a response
     * is vc_cmd_defer. Unspecified for other responses.
     *
     * @return True on success, including if the command is deferred or
     * rejected.  False only on errors.
     */
    bool StateMachine::handle_cmd(nano_t control_time, vehicle_cmd_t cmd,
                                  vc_cmd_response_t &response,
                                  nano_t &retry_at) RUNTIME
    {
        response = vc_cmd_defer;
        retry_at = slate[nominal_transition_time_tok];
        if (slate[pending_cmd_index_tok] != noop_cmd_index)
        {
            /*
             * There is already a pending command.  Tell the caller to
             * call back after the nominal transition time.  This way
             * the pending command will have been handled and we can
             * have another crack at the caller's command.
             */
            return true;
        }
        const std::string &cmd_name = cmd.get_name();
        const uint cmd_index = cmd.get_index();
        if (FswIf(cmd.is_invalid()))
        {
            FswPrefix();
            dbnprintf(200, ": Attempted to issue the '%s' command!\n",
                      cmd_name.c_str());
            response = vc_cmd_reject;
            return false;
        }
        if (FswIfNot(cmd_table.check(cmd)))
        {
            FswPrefix();
            dbnprintf(200,
                      ": Attempted to issue command '%s' to the "
                      "wrong StateMachine!\n",
                      cmd_name.c_str());
            response = vc_cmd_reject;
            return false;
        }
        FswAbortOutsideRange((uint)slate[cur_state_tok], 0U, state.size(),
                             false);
        Handle<RUNTIME ControlState> cs = state[slate[cur_state_tok]];
        FswAbortIfNot(cs, false);
        FswAbortOutsideRange(cmd_index, 0U, cs->cmds.size(), false);
        uint ns = cs->cmds[cmd_index];
        if (timeout_cmd == cmd)
        {
            /*
             * Handle the timeout command as a special case to avoid alarming
             * console messages.
             */
            if (ns == undefined_state)
            {
                response = vc_cmd_reject;
                return true;
            }
            else if (ns == ignore_state)
            {
                response = vc_cmd_accept;
                return true;
            }
        }
        if (dbverbose() >= local_verbosity + 1)
        {
            dbnprintf(200, "%9.6f: SM(%s): cmd: %s(%d)\n",
                      get_rel_time() / dbillion, state_machine_name.c_str(),
                      cmd_name.c_str(), cmd_index);
        }
        if (ns == ignore_state || cmd.is_noop())
        {
            if (dbverbose() >= local_verbosity)
                dbnprintf(200,
                          "%9.6f: SM(%s): cmd: %s(%d) "
                          "ignored in state %s(%d).\n",
                          get_rel_time() / dbillion, state_machine_name.c_str(),
                          cmd_name.c_str(), cmd_index, get_state().c_str(),
                          slate[cur_state_tok]);
            response = vc_cmd_accept;
            return true;
        }
        if (ns == undefined_state)
        {
            FswPrefix();
            dbnprintf(200, ": SM(%s): cmd %s(%d) is invalid in state %s(%d).\n",
                      state_machine_name.c_str(), cmd_name.c_str(), cmd_index,
                      get_state().c_str(), slate[cur_state_tok]);
            response = vc_cmd_reject;
            return true;
        }
        if (ns >= state.size())
        {
            FswPrefix();
            dbnprintf(
                200, ": SM(%s): cmd %s(%d) leads to undefined state: %d\n",
                state_machine_name.c_str(), cmd_name.c_str(), cmd_index, ns);
            response = vc_cmd_reject;
            return true;
        }
        /*
         * Accept the command and make it the pending command.
         *
         * We set the nominal_transition_time to the previous nominal cycle
         * time.
         */
        FswAssert(slate[pending_cmd_index_tok] == noop_cmd_index);
        slate[pending_cmd_index_tok] = cmd.get_index();
        response = vc_cmd_accept;
        FswAbortIfNot(set_nominal_transition_time(control_time), false);
        return true;
    }
    /**
     * Lookup the command corresponding to the supplied name.
     *
     * @param name The name to lookup.
     * @param[out] cmd Returns the command.
     *
     * @return True on success.
     */
    bool StateMachine::lookup_cmd(const std::string &name,
                                  vehicle_cmd_t &cmd) const
    {
        if (!(cmd_table.lookup(name, cmd)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown command name: %s\n", name.c_str());
            return false;
        }
        return true;
    }
    /**
     * Lookup the command corresponding to the supplied index.
     *
     * @param cmd_index The command index to lookup.
     * @param[out] cmd Returns the command.
     *
     * @return True on success.
     */
    bool StateMachine::lookup_cmd(const uint cmd_index,
                                  vehicle_cmd_t &cmd) const
    {
        if (!cmd_table.lookup(cmd_index, cmd))
        {
            FswPrefix();
            dbnprintf(200, ": unknown command index: %u\n", cmd_index);
            return false;
        }
        return true;
    }
    /**
     * Determine if the StateMachine has a given state. Useful for validating
     * config files.
     *
     * @param name The name of the state to inquire about.
     *
     * @return True if the state exists.
     */
    bool StateMachine::has_state(const std::string &name) const
    {
        uint s = 0;
        return state_sym.raw_get(name, s);
    }
    /**
     * Determine if the StateMachine has any state that accepts the given
     * command (handled or ignored). This always returns true for the noop
     * command as well since it's implicitly accepted always. Useful for
     * validating config files.
     *
     * @param cmd The command to inquire about.
     *
     * @return True if there's a state that has a transition with the
     * given command.
     */
    bool StateMachine::has_cmd(vehicle_cmd_t cmd) const
    {
        if (cmd.is_noop())
        {
            return true;
        }
        const uint cmd_index = cmd.get_index();
        for (size_t i = 0; i < state.size(); i++)
        {
            if (FswIfNot(state[i]))
            {
                continue;
            }
            FswAbortOutsideRange(cmd_index, 0U, state[i]->cmds.size(), false);
            const uint next_state = state[i]->cmds[cmd_index];
            if (next_state != undefined_state)
            {
                return true;
            }
        }
        return false;
    }
    /**
     * Verify the initial state meets requirements and transition the state
     * machine to its initial state.
     *
     * This is executed at the end of the initialization of the StateMachine.
     *
     * @return True on success.
     */
    bool StateMachine::set_initial_state()
    {
        /*
         * Double-check expected initial conditions.
         */
        FswAbortIf(is_init, false);
        FswAbortIfNot(is_init_storage, false);
        FswAbortIf(state.empty(), false);
        FswAbortIfNeqInt(slate[cur_state_tok], 0, false);
        FswAbortIfNeqDouble(slate[event_sequence_time_tok], 0.0, false);
        FswAbortIfNeqUint(slate[pending_cmd_index_tok], noop_cmd_index, false);
        FswAbortIfNeqInt64(slate[nominal_transition_time_tok], nano_t_max,
                           false);
        /*
         * Validate the initial state.
         */
        Handle<ControlState> init_state = state[0];
        const std::vector<ControlState::task_info> &tasks = init_state->tasks;
        const std::vector<uint> &cmds = init_state->cmds;
        /*
         * The initial state has only zero-length event sequences (or no
         * sequences at all).
         */
        FswMsgAbortIfNot(
            init_state->duration == 0LL, false, 100,
            "If an event sequence is specified for the initial state, "
            "its duration needs to be 0ms.");
        /*
         * The initial state does not start, command, continue, or stop any
         * control tasks.
         */
        for (size_t i = 0; i < tasks.size(); i++)
        {
            FswMsgAbortIf(
                tasks[i].is_init, false, 100,
                "The initial state cannot start, command, continue, or stop "
                "any control tasks.");
        }
        /*
         * If a timeout command is specified, no other command can be.
         */
        if (!init_state->timeout_ignored)
        {
            const uint timeout_index = timeout_cmd.get_index();
            for (size_t i = 0; i < cmds.size(); i++)
            {
                if (i != timeout_index)
                {
                    FswMsgAbortIfNot(
                        cmds[i] == undefined_state, false, 100,
                        "No command can be specified for the initial state "
                        "if a timeout command already is.");
                }
            }
        }
        /*
         * The initial state meets all requirements. Now process any potential
         * EventSequence associated with it.
         */
        if (!init_state->seqs.empty())
        {
            /*
             * We do not need to compute and set the nominal transition time
             * because:
             * - The initial state duration is always 0ms.
             * - We process the initial state event sequence immediately during
             *   initialization of the vehicle.
             */
            const nano_t transition_time = initial_control_time;
            /*
             * Start and dispatch the initial sequence.
             */
            FswAbortIfNot(init_state->start_seqs(transition_time), false);
            init_state->dispatch_seqs(transition_time);
        }
        /*
         * Inform interested clients about the state change.
         */
        std::string state_name = lookup_state(0);
        state_sig.emit(state_name);
        return true;
    }
    /**
     * Lookup the state number corresponding to a state name.
     *
     * @param name The name to lookup.
     * @param[out] s Returns the state number.
     *
     * @return True on success.
     */
    bool StateMachine::lookup_state(const std::string &name, uint &s) const
    {
        if (FswIfNot(state_sym.raw_get(name, s)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown state name: %s\n", name.c_str());
            return false;
        }
        return true;
    }
    /**
     * Lookup a state number and return the name of the state (or "unknown" if
     * no name is found).
     *
     * @param s The state number.
     *
     * @return The state name.
     */
    std::string StateMachine::lookup_state(uint s) const
    {
        std::string name;
        if (FswIfNot(state_sym.raw_get(s, name)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown state number: %d\n", s);
            return "unknown";
        }
        return name;
    }
    /**
     * Lookup the number of a ControlTask corresponding to the given name.
     *
     * @param name The name of the task.
     * @param[out] ctask Returns the control task number.
     *
     * @return True on success.
     */
    bool StateMachine::lookup_ctask(const std::string &name, uint &ctask) const
    {
        if (FswIfNot(ctask_sym.raw_get(name, ctask)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown control task name: %s\n", name.c_str());
            return false;
        }
        return true;
    }
    /**
     * Lookup the name of a ControlTask from its task number.
     *
     * @param ctask The task number.
     *
     * @return The name of the task or "unknown" if no name is found.
     */
    std::string StateMachine::lookup_ctask(uint ctask) const
    {
        std::string name;
        if (FswIfNot(ctask_sym.raw_get(ctask, name)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown control task number: %d\n", ctask);
            return "unknown";
        }
        return name;
    }
    /**
     * Lookup the control task state number (from the ctask_state_t enum)
     * corresponding to the supplied name.
     *
     * @param name The name to lookup.
     * @param[out] cstate Returns the control task state number (from the
     * ctask_state_t enum).
     *
     * @return True on success.
     */
    bool StateMachine::lookup_cstate(const std::string &name,
                                     ctask_state_t &cstate) const
    {
        uint cstate_index = 0;
        if (FswIfNot(ctask_state_t_sym.raw_get(name + "_cstate", cstate_index)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown control task state name: %s\n",
                      name.c_str());
            return false;
        }
        cstate = static_cast<ctask_state_t>(cstate_index);
        return true;
    }
    /**
     * Lookup the name of the control task state from its number (from the
     * ctask_state_t enum).
     *
     * @param cstate The number to lookup (from the ctask_state_t enum).
     *
     * @return The name or "unknown" if no name is found.
     */
    std::string StateMachine::lookup_cstate(ctask_state_t cstate) const
    {
        std::string name;
        if (FswIfNot(ctask_state_t_sym.raw_get(cstate, name)))
        {
            FswPrefix();
            dbnprintf(200, ": unknown control task state number: %d\n", cstate);
            return "unknown";
        }
        uint us = name.find_last_of('_');
        name = name.substr(0, us);
        return name;
    }
    /**
     * Parse the multi_seq_devices file for this state machine (if it exists)
     * and return mappings from state name to the device and state registry IDs
     * that are allowed to be commanded in multiple event sequences within each
     * state.
     *
     * @param builder The SlateBuilder
     * @param configs Config file finder.
     * @param _role The role name of the state machine.
     * @param[out] multi_seq_devices A mapping from state name to a set of the
     * device IDs for devices that are allowed to be commanded in multiple
     * event sequences within each state.
     * multiple event sequences within each state.
     *
     * @return True on success.
     */
    bool StateMachine::parse_multi_seq_devices(SlateBuilder &builder,
                                               const Configs &configs,
                                               const std::string &_role,
                                               str_str_s_m &multi_seq_devices)
    {
        std::string config_file;
        if (!configs.config_file(_role + ".multi_seq_devices", config_file,
                                 false /*report_error*/))
        {
            return true;
        }
        str_str_v_v_m multi_seq_device_map;
        FswAbortIfNot(read_sections_file(config_file, multi_seq_device_map),
                      false);
        /*
         * Each section in the file consists of a state name (in brackets)
         * followed by a list of device and/or state registry entry names that
         * are allowed to be commanded in multiple event sequences in that
         * state, one per line. Iterate through the file and build up the
         * multi_seq_devices map.
         */
        for (const auto &[state_name, devices] : multi_seq_device_map)
        {
            if (FswIfNot(has_state(state_name)))
            {
                FswPrefix();
                dbnprintf(200, ": Unknown state '%s' in %s.multi_seq_devices\n",
                          state_name.c_str(), _role.c_str());
                return false;
            }
            for (str_v_v::const_iterator j = devices.begin();
                 j != devices.end(); ++j)
            {
                if (FswIfNeq(j->size(), 1))
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": Invalid formatting for state '%s' in "
                              "%s.multi_seq_devices\n",
                              state_name.c_str(), _role.c_str());
                    return false;
                }
                const std::string &device_name = j->front();
                if (FswIfNot(builder.path_exists(device_name)))
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": Unable to lookup device '%s' in "
                              "%s.multi_seq_devices\n",
                              device_name.c_str(), _role.c_str());
                    return false;
                }
                multi_seq_devices[state_name].insert(device_name);
            }
        }
        return true;
    }
    /**
     * Read the description of a supplied state from the configuration files
     * and initialize storage for all required objects. In particular, create
     * EventSequence objects by parsing a state configuration file.
     *
     * @param configs Config file finder.
     * @param builder The slate where to store elements.
     * @param state_num The state to read about.
     *
     * @return True on success.
     */
    bool StateMachine::initialize_state_from_file(const Configs &configs,
                                                  SlateBuilder builder,
                                                  uint state_num)
    {
        FswAbortOutsideRange(state_num, 0U, state_v.size(), false);
        FswAbortOutsideRange(state_num, 0U, state.size(), false);
        const std::string state_name = state_v[state_num];
        Handle<ControlState> cs = state[state_num];
        /*
         * Lookup the name and find the configuration file.
         */
        std::string sfilename;
        FswAbortIfNot(configs.config_file(state_name + ".state", sfilename),
                      false);
        /*
         * We do not use the state name for the sub-slate because it can be
         * arbitrary long, which can break compatibility with SlateDevice.
         */
        SlateBuilder sub = builder.sub_slate("sm_" + role)
                               .sub_slate("state_" + to_string(state_num + 1));
        StdioFile sfile(sfilename);
        FILE *in;
        FswAbortIfNot(sfile.open("r", in), false);
        str_v words;
        bool seq_seen = false;
        while (get_line_words(in, words, true))
        {
            FswAssert(words.size() > 0);
            FswAssert(words[0].size() > 0);
            if (words[0][0] == '#')
            {
                /*
                 * Comment.
                 */
                continue;
            }
            if (words[0] == "seq")
            {
                if (seq_seen)
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": State file %s contains multiple seq "
                              "directives, and may only contain one.\n",
                              sfile.name().c_str());
                    return false;
                }
                else
                {
                    seq_seen = true;
                }
                if (words.size() < 2)
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": State file %s contains an empty "
                              "seq line.\n",
                              sfile.name().c_str());
                    return false;
                }
                /*
                 * Read each sequence file. They will be validated after the
                 * the entire event file is read.
                 */
                for (size_t i = 1; i < words.size(); ++i)
                {
                    if (cs->seqs.count(words[i]) == 1)
                    {
                        FswPrefix();
                        dbnprintf(200,
                                  ": Sequence file %s has already been "
                                  "added.\n",
                                  words[i].c_str());
                        return false;
                    }
                    Handle<EventSequence> &es = cs->seqs[words[i]];
                    es.assume_ownership(new EventSequence);
                    FswAbortIfNot(es, false);
                    FswAbortIfNot(
                        es->init_storage(sub.sub_slate("seq_" + to_string(i))),
                        false);
                }
            }
            else if (words[0] == "cmd")
            {
                /*
                 * will parse later in finalize_state_from_file().
                 */
                continue;
            }
            else if (words[0] == "task")
            {
                /*
                 * will parse later in finalize_state_from_file().
                 */
                continue;
            }
            else
            {
                FswPrefix();
                dbnprintf(200, ": Unknown keyword \"%s\" in file: %s\n",
                          words[0].c_str(), sfile.name().c_str());
                return false;
            }
        }
        return true;
    }
    /**
     * Parses the command and state from the input words.
     *
     * @param words The words to parse. Contains a command and new state.
     * @param cmd The command
     * @param new_state The new state
     *
     * @return True on success
     */
    bool StateMachine::parse_state_transitions(const str_v &words,
                                               vehicle_cmd_t &cmd,
                                               uint &new_state) const
    {
        FswAbortIfNeqInt(words.size(), 3, false);
        FswAbortIfNot(lookup_cmd(words[1], cmd), false);
        FswAbortIfNot(lookup_state(words[2], new_state), false);
        return true;
    }
    /**
     * Read the description of a supplied state from the configuration
     * files, parse commands and tasks.
     *
     * @param configs Config file finder.
     * @param state_num The state to read about.
     * @param multi_seq_devices A mapping from state name to a set of the
     * device IDs for devices that are allowed to be commanded in multiple
     * event sequences within each state.
     * multiple event sequences within each state.
     *
     * @return True on success.
     */
    bool
    StateMachine::finalize_state_from_file(const Configs &configs,
                                           uint state_num,
                                           const str_str_s_m &multi_seq_devices)
    {
        FswAbortOutsideRange(state_num, 0U, state_v.size(), false);
        FswAbortOutsideRange(state_num, 0U, state.size(), false);
        const std::string state_name = state_v[state_num];
        Handle<ControlState> cs = state[state_num];
        /*
         * Lookup the name and find the configuration file.
         */
        std::string sfilename;
        FswAbortIfNot(configs.config_file(state_name + ".state", sfilename),
                      false);
        StdioFile sfile(sfilename);
        FILE *in;
        FswAbortIfNot(sfile.open("r", in), false);
        vehicle_cmd_t run_alarm_event_sequence_cmd;
        FswAbortIfNot(lookup_cmd("run_alarm_event_sequence",
                                 run_alarm_event_sequence_cmd),
                      false);
        str_v words;
        while (get_line_words(in, words, true))
        {
            FswAssert(words.size() > 0);
            FswAssert(words[0].size() > 0);
            if (words[0] == "cmd")
            {
                /*
                 * Define a command.
                 */
                vehicle_cmd_t cmd;
                uint new_state;
                FswAbortIfNot(parse_state_transitions(words, cmd, new_state),
                              false);
                const uint cmd_index = cmd.get_index();
                FswAssert(cmd_index < cs->cmds.size());
                /*
                 * It is not possible to command back to the initial state.
                 */
                FswMsgAbortIf(new_state == 0, false, 100,
                              "It is not possible to transition back to the "
                              "initial state.");
                /*
                 * Neither invalid commands nor noop commands may cause
                 * state transitions.
                 *
                 * run_alarm_event_sequence is only for use by
                 * multi-sensor alarms and AlarmEventSequenceHandler; it
                 * cannot be used to cause a state transition.
                 */
                if (cmd.is_invalid() || cmd.is_noop() ||
                    cmd == run_alarm_event_sequence_cmd)
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": Command '%s' may not participate in "
                              "a state transition.\n",
                              cmd.get_name().c_str());
                    return false;
                }
                if (cs->cmds[cmd_index] != undefined_state)
                {
                    FswPrefix();
                    dbnprintf(200, ": Duplicate command \"%s\" in file: %s\n",
                              words[1].c_str(), sfile.name().c_str());
                    return false;
                }
                cs->cmds[cmd_index] = new_state;
            }
            else if (words[0] == "task")
            {
                /*
                 * Define a task and its state.
                 */
                uint t;
                ctask_state_t t_state;
                FswAbortIfNotOpInt(words.size(), >=, 3, false);
                /*
                 * Look up the task number and the state number.
                 */
                FswAbortIfNot(lookup_ctask(words[1], t), false);
                FswAbortIfNot(lookup_cstate(words[2], t_state), false);
                /*
                 * Get the task info.
                 */
                FswAssert(t < cs->tasks.size());
                ControlState::task_info &ti = cs->tasks[t];
                /*
                 * Check that this task has not already been listed in this
                 * state file.
                 */
                if (ti.is_init)
                {
                    FswPrefix();
                    dbnprintf(400, ": Task \"%s\" already listed in file: %s\n",
                              words[1].c_str(), sfile.name().c_str());
                    return false;
                }
                /*
                 * The "off" and "cont" commands do not accept parameters.
                 */
                if (t_state == off_cstate || t_state == cont_cstate)
                {
                    FswAbortIfNeq(words.size(), 3, false);
                }
                if (t_state == cmd_cstate || t_state == on_cstate)
                {
                    for (size_t i = 3; i < words.size(); ++i)
                    {
                        /*
                         * Save additional arguments.
                         */
                        ti.args.push_back(words[i]);
                    }
                }
                /*
                 * Save the task state and mark this task as initialized.
                 */
                ti.state = t_state;
                ti.is_init = true;
            }
            /*
             * Other possible values for words[0] have already been validated
             * in initialize_state_from_file().
             */
        }
        /*
         * Determine if timeout is ignored, and store it in this state.
         */
        FswAssert("timeout" == timeout_cmd.get_name());
        const uint timeout_index = timeout_cmd.get_index();
        cs->timeout_ignored = (cs->cmds[timeout_index] == ignore_state) ||
                              (cs->cmds[timeout_index] == undefined_state);
        if (!cs->timeout_ignored)
        {
            /*
             * A timeout command requires an EventSequence.
             */
            FswMsgAbortIf(cs->seqs.empty(), false, 100,
                          "A timeout command cannot be specified without an "
                          "EventSequence.");
        }
        /*
         * If entries exist in the multi_seq_devices and/or multi_seq_sr_entries
         * maps, associate them with the ControlState object.
         */
        str_str_s_m::const_iterator multi_seq_devices_iter =
            multi_seq_devices.find(state_name);
        if (multi_seq_devices_iter != multi_seq_devices.end())
        {
            cs->multi_seq_devices = multi_seq_devices_iter->second;
        }
        return true;
    }
    /**
     * Performs validation on this state.
     *
     * @param cs The current control state being verified.
     * @param state_idx The state number of this control state.
     *
     * @return True if all verification succeeded.
     */
    bool StateMachine::validate_state(const ControlState &cs,
                                      const uint state_idx) const
    {
        FswAbortIfNeq(cs.tasks.size(), task.size(), false);
        const bool is_initial_state = (state_idx == 0);
        for (size_t task_idx = 0; task_idx < task.size(); task_idx++)
        {
            const ControlState::task_info &ti = cs.tasks[task_idx];
            Handle<RUNTIME ControlTask> ctask = task[task_idx];
            FswAbortIfNot(ctask, false);
            switch (ti.state)
            {
            /*
             * Validate control task commands.
             */
            case cmd_cstate:
                FswAbortIfNot(ctask->validate_cmd(ti.args, cmd_table, cs.cmds),
                              false);
                FswAbortIfNot(ti.args.size() >= 1, false);
                break;
            /*
             * Validate control task on commands.
             */
            case on_cstate:
                FswAbortIfNot(
                    ctask->validate_start(ti.args, cmd_table, cs.cmds), false);
                break;
            /*
             * Validate control task off commands.
             */
            case off_cstate:
                /*
                 * Nothing can be stopped in the initial state.
                 * Per set_initial_state(), the initial state cannot start,
                 * stop, or command tasks.
                 * This guard is important insofar as assertions of the form,
                 * "this task should never be stopped", are violated for the
                 * initial state. In bypassing these checks for the initial
                 * state we ensure these checks remain applicable to all other
                 * states.
                 */
                if (!is_initial_state)
                {
                    FswAbortIfNot(ctask->validate_stop(), false);
                }
                break;
            case cont_cstate:
                break;
            default:
                FswPrefix();
                dbnprintf(100, ": Invalid task state: %d in state '%s'\n",
                          ti.state, lookup_state(state_idx).c_str());
                return false;
            }
        }
        return true;
    }
    /**
     * Validate the graph for safety.
     *
     * @details
     * Validate the graph formed by all the states to ensure that it is safe
     * to dispatch. We make sure there are no cycles of dangerous states. Zero
     * length (or less-than-a-cycle length) states are always considered
     * dangerous, and single-cycle length states are dangerous for some state
     * machine types (see \ref allow_single_cycle).
     *
     * @param allow_single_cycle If this is true (as it is for async state
     *  machines) then single cycle states are not considered dangerous. This
     *  will be set to false for sync state machines because single-cycle
     *  states do not allow external commands because they always would have a
     *  pending command.
     *
     * @note If you change this behavior you need to re-validate SubStateMachine
     * which is dependent on no cycles in the state graph.
     *
     * @return True on success.
     */
    bool StateMachine::validate_graph(bool allow_single_cycle) const
    {
        FswAssert("timeout" == timeout_cmd.get_name());
        const uint timeout_index = timeout_cmd.get_index();
        /*
         * Build up the set of states that are potentially dangerous.
         */
        std::set<uint> dangerous_states;
        for (uint state_idx = 0; state_idx < state.size(); state_idx++)
        {
            Handle<RUNTIME ControlState> cs = state[state_idx];
            FswAssert(cs);
            FswAssert(timeout_index < cs->cmds.size());
            if ((!cs->seqs.empty()) &&
                ((cs->duration < transition_period) ||
                 (!allow_single_cycle &&
                  (cs->duration == transition_period))) &&
                (cs->cmds[timeout_index] < state.size()))
            {
                dangerous_states.insert(state_idx);
            }
        }
        /*
         * Explore the graph from each of the dangerous states and see if
         * there's a cycle that ends up at a dangerous state *that only
         * goes through dangerous states*.
         */
        std::vector<color> colors(state.size(), unexplored);
        for (std::set<uint>::const_iterator iter = dangerous_states.begin();
             iter != dangerous_states.end(); ++iter)
        {
            const uint state_idx = *iter;
            FswAssert(state_idx < colors.size());
            if (colors[state_idx] == unexplored)
            {
                bool cycle_detected = false;
                FswAbortIfNot(search_for_cycle(dangerous_states, colors,
                                               cycle_detected, state_idx),
                              false);
                if (cycle_detected)
                {
                    FswPrefix();
                    dbnprintf(200,
                              ": A loop of states was detected starting "
                              "at state '%s' which would not terminate or not "
                              "be externally commandable.\n",
                              lookup_state(state_idx).c_str());
                    return false;
                }
            }
        }
        return true;
    }
    /**
     * Look for a cycle consisting of all dangerous states, starting at a given
     * state. This does a depth first search algorithm.
     *
     * @details Dangerous states are ones with an event sequence that's less
     * than the transition period and where the timeout command goes to
     * another state.
     *
     * @param dangerous_states A set with indexes of all the dangerous
     * states.
     * @param[in,out] colors List of node colors based on the search so far.
     * @param[out] cycle_detected Will be set to true if a cycle is found.
     * @param state_idx Index of the state to start search at.
     *
     * @return True on success.
     */
    bool StateMachine::search_for_cycle(const std::set<uint> &dangerous_states,
                                        std::vector<color> &colors,
                                        bool &cycle_detected,
                                        uint state_idx) const
    {
        FswAssert(state_idx < colors.size());
        FswAssert(colors[state_idx] == unexplored);
        /*
         * Non-dangerous states break any cycle and don't need to be explored
         * further.
         */
        if (dangerous_states.find(state_idx) == dangerous_states.end())
        {
            colors[state_idx] = explored;
            return true;
        }
        /*
         * This might be part of a dangerous cycle - we need to explore it.
         */
        colors[state_idx] = in_progress;
        /*
         * See where this goes on a timeout. If the timeout ends up going
         * to a dangerous state that we were in the process of exploring then
         * we have detected a cycle.
         */
        FswAssert("timeout" == timeout_cmd.get_name());
        const uint timeout_index = timeout_cmd.get_index();
        FswAssert(state_idx < state.size());
        Handle<RUNTIME ControlState> cs = state[state_idx];
        FswAssert(cs);
        FswAssert(timeout_index < cs->cmds.size());
        const uint next_state_idx = cs->cmds[timeout_index];
        if (next_state_idx < state.size())
        {
            FswAssert(next_state_idx < colors.size());
            const color timeout_color = colors[next_state_idx];
            if (timeout_color == in_progress)
            {
                FswAssert(dangerous_states.find(next_state_idx) !=
                          dangerous_states.end());
                cycle_detected = true;
                return true;
            }
            else if (timeout_color == unexplored)
            {
                FswAbortIfNot(search_for_cycle(dangerous_states, colors,
                                               cycle_detected, next_state_idx),
                              false);
            }
            else
            {
                FswAssert(timeout_color == explored);
            }
        }
        /*
         * This one looks ok.
         */
        colors[state_idx] = explored;
        return true;
    }
    /**
     * Handle the actual state transition for the pending command.
     *
     * @note
     * Most of the work is done by handle_pending_cmd_aux(). Regardless of
     * whether it encounters an error, we clear the pending command prior to
     * returning.
     *
     * @param transition_time Time that the transition took place (may be
     * back-dated).
     * @param transition_pre  If true we will transition the pre tasks as we
     * set the state. For async state machines this should always be true and
     * for sync state machines this should be false for partial transitions.
     * @param transition_post If true we will transition the post tasks as
     * we set the state. For async state machines this should always be true and
     * for sync state machines this should always be false.
     * @param start_seq If true we will start the next state's event
     * sequence as we set the state. For async state machines this should always
     * be true and for sync state machines this is false for partial
     * transitions.
     *
     * @return True on success.
     */
    bool StateMachine::handle_pending_cmd(nano_t transition_time,
                                          bool transition_pre,
                                          bool transition_post,
                                          bool start_seq) RUNTIME
    {
        FswAssert(slate[pending_cmd_index_tok] != noop_cmd_index);
        bool ret = true;
        FswIfNot2(handle_pending_cmd_aux(transition_time, transition_pre,
                                         transition_post, start_seq),
                  ret);
        /*
         * Regardless of the success or failure of handle_pending_cmd,
         * we want to make sure that we clear pending_cmd and
         * nominal_transition_time, so that we don't just retry it
         * immediately.
         */
        slate[pending_cmd_index_tok] = noop_cmd_index;
        slate[nominal_transition_time_tok] = nano_t_max;
        return ret;
    }
    /**
     * Handle the actual state transition for the pending command.
     *
     * @note Does most of the actual work for handle_pending_cmd().  If an error
     * is encountered, we just abort.
     *
     * @param transition_time Time that the transition took place (may be
     * back-dated).
     * @param transition_pre  If true we will transition the pre tasks as we
     * set the state. For async state machines this should always be true and
     * for sync state machines this should be false for partial transitions.
     * @param transition_post If true we will transition the post tasks as
     * we set the state. For async state machines this should always be true and
     * for sync state machines this should always be false.
     * @param start_seq If true we will start the next state's event
     * sequence as we set the state. For async state machines this should always
     * be true and for sync state machines this is false for partial
     * transitions.
     *
     * @return True on success.
     */
    bool StateMachine::handle_pending_cmd_aux(nano_t transition_time,
                                              bool transition_pre,
                                              bool transition_post,
                                              bool start_seq) RUNTIME
    {
        FswAbortIfNot(transition_time != nano_t_max, false);
        FswAbortIf(slate[pending_cmd_index_tok] == noop_cmd_index, false);
        FswAbortOutsideRange((uint)slate[cur_state_tok], 0U, state.size(),
                             false);
        Handle<RUNTIME ControlState> cs = state[slate[cur_state_tok]];
        FswAbortIfNot(cs, false);
        const uint pending_index = slate[pending_cmd_index_tok];
        FswAbortOutsideRange(pending_index, 0U, cs->cmds.size(), false);
        uint ns = cs->cmds[pending_index];
        /*
         * The handle_cmd function should have dealt with undefined_state,
         * ignored_state, or invalid states already, so these aborts are not
         * expected to trigger.
         */
        FswAbortIf(ns == undefined_state, false);
        FswAbortIf(ns == ignore_state, false);
        FswAbortIf(ns >= state.size(), false);
        /*
         * Actually set the new state.
         */
        FswAbortIfNot(set_state(ns, slate[pending_cmd_index_tok],
                                transition_time, transition_pre,
                                transition_post, start_seq),
                      false);
        return true;
    }
    /**
     * Transition to a new state starting and stopping ControlTasks as
     * needed.
     *
     * @param new_state_num The new state.
     * @param cmd_index The index of the command causing the state transition.
     * Use the noop_cmd_index value to indicate that the transition was not
     * triggered by a command.
     * @param transition_time Time that the transition took place (may be back-
     * dated).
     * @param transition_pre If true we will transition the pre tasks as we set
     * the state. For async state machines this should always be true and for
     * sync state machines this should be false for partial transitions.
     * @param transition_post If true we will transition the post tasks as we
     * set the state. For async state machines this should always be true and
     * for sync state machines this should always be false.
     * @param start_seq If true we will start the next state's event sequence as
     * we set the state. For async state machines this should always be true and
     * for sync state machines this is false for partial transitions.
     *
     * @return True on success.
     */
    bool StateMachine::set_state(uint new_state_num, uint cmd_index,
                                 nano_t transition_time, bool transition_pre,
                                 bool transition_post, bool start_seq) RUNTIME
    {
        FswAbortIfNot(is_init, false);
        FswAssert(transition_time != nano_t_max);
        const uint current_state = slate[cur_state_tok];
        if (dbverbose() >= local_verbosity)
        {
            vehicle_cmd_t cmd;
            FswAbortIfNot(lookup_cmd(cmd_index, cmd), false);
            dbnprintf(200, "%9.6f: SM(%s): cmd %s(%d): %s(%d) -> %s(%d)",
                      get_rel_time() / dbillion, state_machine_name.c_str(),
                      cmd.get_name().c_str(), cmd.get_index(),
                      lookup_state(current_state).c_str(), current_state,
                      lookup_state(new_state_num).c_str(), new_state_num);
            /*
             * Note that we always use the nominal transition time here, which
             * might be back-dated. This works for the sync and async flavors.
             */
            dbnprintf(200, " t=%9.6f\n",
                      (slate[nominal_transition_time_tok] - get_timebase()) /
                          dbillion);
        }
        /*
         * We should only be getting asked to set a real state.
         */
        FswAbortOutsideRange(new_state_num, 0U, state.size(), false);
        Handle<RUNTIME ControlState> ncs = state[new_state_num];
        FswAbortIfNot(ncs, false);
        /*
         * It's pretty nasty that we have this boolean interface here but it was
         * the simplest way to keep the old async behavior working without
         * having to restructure this in unnatural ways. This also lets us reuse
         * this function when setting the initial state in a way that works for
         * both flavors.
         */
        bool failed = false;
        /*
         * Transition pre tasks for the new state.
         */
        if (transition_pre &&
            FswIfNot(transition_pre_tasks(transition_time, ncs)))
        {
            failed = true;
        }
        /*
         * Transition post tasks for the new state.
         */
        if (transition_post &&
            FswIfNot(transition_post_tasks(transition_time, ncs)))
        {
            failed = true;
        }
        /*
         * End the previous sequence.
         * This is to make sure all EventSequence start_time are the same across
         * all strings for previously executed event sequences (which might not
         * be the case if they have executed before strings are in sync).
         */
        if ((new_state_num != current_state) &&
            FswIfNot(state[current_state]->end_seqs()))
        {
            failed = true;
        }
        /*
         * Start the new sequence.
         */
        if (start_seq && !ncs->seqs.empty() &&
            FswIfNot(ncs->start_seqs(transition_time)))
        {
            failed = true;
        }
        /*
         * Record the current and old state machine states. Also set event
         * sequence time to zero; this makes sure the event_sequence_time is
         * reset for states that don't have event sequences.
         */
        slate[older_state_tok] = slate[old_state_tok];
        slate[old_state_tok] = slate[cur_state_tok];
        slate[event_sequence_time_tok] = 0.0;
        slate[cur_state_tok] = static_cast<INT32>(new_state_num);
        /*
         * Inform interested clients about the state change.
         */
        std::string state_name = lookup_state(slate[cur_state_tok]);
        state_sig.emit(state_name);
        return !failed;
    }
    /**
     * Transition a task to the appropriate new task state for the
     * supplied state machine state.
     *
     * @param transition_time Nominal transition time (might be back-dated).
     * @param task_num The task.
     * @param cs The state machine new state's ControlState.
     *
     * @return True on success.
     */
    bool StateMachine::transition_task(nano_t transition_time, uint task_num,
                                       Handle<RUNTIME ControlState> cs) RUNTIME
    {
        FswAbortOutsideRange(task_num, 0U, task.size(), false);
        FswAbortIfNot(cs, false);
        FswAbortOutsideRange(task_num, 0U, cs->tasks.size(), false);
        Handle<RUNTIME ControlTask> ct = task[task_num];
        FswAbortIfNot(ct, false);
        const ControlState::task_info &ti = cs->tasks[task_num];
        FswAbortOutsideRange(ti.state, off_cstate, num_cstate, false);
        FswAssert(transition_time != nano_t_max);
        switch (ti.state)
        {
        case on_cstate: {
            if (ct->start(transition_time, ti.args))
            {
                return true;
            }
            break;
        }
        case off_cstate: {
            if (ct->stop(transition_time))
            {
                return true;
            }
            break;
        }
        case cont_cstate: {
            if (ct->cont(transition_time))
            {
                return true;
            }
            break;
        }
        case cmd_cstate: {
            if (ct->cmd(transition_time, ti.args))
            {
                return true;
            }
            break;
        }
        default: {
            /*
             * Error handled below.
             */
            break;
        }
        }
        FswPrefix();
        dbnprintf(200, ": Failed setting task \"%s\" to \"%s\"\n",
                  lookup_ctask(task_num).c_str(),
                  lookup_cstate(ti.state).c_str());
        if (ti.state == cmd_cstate || ti.state == on_cstate)
        {
            FswPrefix();
            dbnprintf(200, ": args:");
            for (uint i = 0; i < ti.args.size(); i++)
            {
                dbnprintf(200, " \"%s\"", ti.args[i].c_str());
            }
            dbnprintf(200, "\n");
        }
        return false;
    }
    /**
     * Transition all the pre-transition control tasks.
     *
     * @param transition_time Nominal transition time (might be back-dated).
     * @param cs Control state context to transition in.
     *
     * @return True on success.
     */
    bool
    StateMachine::transition_pre_tasks(nano_t transition_time,
                                       Handle<RUNTIME ControlState> cs) RUNTIME
    {
        FswAbortIfNot(cs, false);
        bool good = true;
        for (uint i = 0; i < num_pre_transition_ctasks; i++)
        {
            FswIfNot2(transition_task(transition_time, i, cs), good);
        }
        return good;
    }
    /**
     * Transition all the post-transition control tasks.
     *
     * @param transition_time Nominal transition time (might be back-dated).
     * @param cs Control state context to transition in.
     *
     * @return True on success.
     */
    bool
    StateMachine::transition_post_tasks(nano_t transition_time,
                                        Handle<RUNTIME ControlState> cs) RUNTIME
    {
        FswAbortIfNot(cs, false);
        bool good = true;
        for (uint i = num_pre_transition_ctasks; i < task.size(); i++)
        {
            FswIfNot2(transition_task(transition_time, i, cs), good);
        }
        return good;
    }
    /**
     * Set a nominal transition time to the nominal cycle at or
     * preceeding the supplied time.
     *
     * The chosen time will be on a cycle time boundary and will
     * be the latest such time not in the future.
     *
     * @param control_time The current control time.
     *
     * @return True on success.
     */
    bool StateMachine::set_nominal_transition_time(nano_t control_time) RUNTIME
    {
        slate[nominal_transition_time_tok] =
            Periodic::calculate_next_time(control_time, transition_period) -
            transition_period;
        FswAssert(control_time - transition_period <
                  slate[nominal_transition_time_tok]);
        FswAssert(slate[nominal_transition_time_tok] <= control_time);
        return true;
    }
    /**
     * Dispatch all the pre-transition control tasks.
     *
     * @param control_time The current control time.
     *
     * @return The time of the next event we want to be called back for.
     */
    nano_t StateMachine::dispatch_pre_tasks(nano_t control_time) RUNTIME
    {
        nano_t next_event = nano_t_max;
        for (uint i = 0; i < num_pre_transition_ctasks; i++)
        {
            FswAssert(task[i]);
            if (!task[i]->is_active())
            {
                continue;
            }
            const nano_t t = task[i]->dispatch(control_time);
            next_event = std::min(next_event, t);
        }
        return next_event;
    }
    /**
     * Dispatch all the post-transition control tasks.
     *
     * @param control_time The current control time.
     *
     * @return The time of the next event we want to be called back for.
     */
    nano_t StateMachine::dispatch_post_tasks(nano_t control_time) RUNTIME
    {
        nano_t next_event = nano_t_max;
        for (uint i = num_pre_transition_ctasks; i < task.size(); i++)
        {
            FswAssert(task[i]);
            if (!task[i]->is_active())
            {
                continue;
            }
            const nano_t t = task[i]->dispatch(control_time);
            next_event = std::min(next_event, t);
        }
        return next_event;
    }
    /**
     * Constructor.
     *
     * @param _cmd_table Reference to the CommandTable. Used to look
     * up vehicle commands.
     * @param _transition_period The period at which to check for state
     * transitions.
     * @param _local_verbosity Affects what dbverbose() level is required
     * before debug messages are printed.
     * @param _initial_control_time Time to use to initialize the initial event
     * sequence.
     */
    AsyncStateMachine::AsyncStateMachine(const CommandTable &_cmd_table,
                                         const nano_t _transition_period,
                                         const int _local_verbosity,
                                         const nano_t _initial_control_time)
        : StateMachine(_cmd_table, _transition_period, _local_verbosity,
                       _initial_control_time)
    {}
    /**
     * Destructor.
     */
    AsyncStateMachine::~AsyncStateMachine() {}
    /**
     * Dispatch any scheduled events.
     *
     * @param control_time The current control time.
     *
     * @return The time of the next event we want to be called back for.
     */
    nano_t AsyncStateMachine::dispatch(nano_t control_time) RUNTIME
    {
        nano_t next_event = nano_t_max;
        /*
         * First we dispatch any pre-transition control tasks.
         */
        {
            const nano_t t = dispatch_pre_tasks(control_time);
            next_event = std::min(next_event, t);
        }
        /*
         * Now we check for pending state transitions (if the time is correct to
         * do so). (Note that next_event is not updated based on
         * nominal_transition_time here. See below.)
         */
        if (slate[nominal_transition_time_tok] <= control_time)
        {
            FswIfNot(handle_pending_cmd(
                slate[nominal_transition_time_tok], true /*transition_pre*/,
                true /*transition_post*/, true /*start_seq*/));
            /*
             * Pre-transition control tasks have already given us their next
             * wake up time, but we might have invalidated it by changing
             * states, so we ask for an immediate callback in the new mode.
             */
            next_event = nano_t_min;
        }
        /*
         * Get the current state, which might have just gotten transitioned to.
         */
        FswAbortOutsideRange((uint)slate[cur_state_tok], 0U, state.size(),
                             next_event);
        Handle<RUNTIME ControlState> cs = state[slate[cur_state_tok]];
        FswAbortIfNot(cs, next_event);
        /*
         * Dispatch the event sequence if there is one, and look for a timeout
         * if it has reached the end.
         */
        if (!cs->seqs.empty())
        {
            nano_t t = cs->dispatch_seqs(control_time);
            next_event = std::min(next_event, t);
            /*
             * Save off the current time into the current sequence for
             * telemetry.
             */
            slate[event_sequence_time_tok] = cs->get_current_time() / dbillion;
            if (t == nano_t_max)
            {
                /*
                 * The sequence has ended.
                 *
                 * If there's a timeout action for a new state, then we want to
                 * timeout and redispatch asap.
                 *
                 * If there's no timeout action, then we continue looking for
                 * other events.
                 */
                vc_cmd_response_t response = vc_cmd_reject;
                nano_t retry_at = nano_t_max;
                bool good = true;
                FswIfNot2(
                    handle_cmd(control_time, timeout_cmd, response, retry_at),
                    good);
                if (good && response == vc_cmd_accept &&
                    (slate[pending_cmd_index_tok] != noop_cmd_index))
                {
                    /*
                     * The timeout command was accepted and resulted in a state
                     * transition.  Request that we be called back immediately
                     * so that we can transition to the new state.  We will
                     * continue with the rest of the dispatch function, however
                     * before returning.
                     */
                    next_event = nano_t_min;
                }
            }
        }
        /*
         * Now dispatch the tasks which run after the transition check and store
         * the time of the earliest next event.
         */
        {
            const nano_t t = dispatch_post_tasks(control_time);
            next_event = std::min(next_event, t);
        }
        /*
         * Update next_event based on nominal_transition_time.  Since
         * nominal_transition_time is set in the past when a command is pending,
         * this will have the effect of requesting an immediate wakeup in that
         * case.
         */
        next_event = std::min(next_event, slate[nominal_transition_time_tok]);
        /*
         * Return the time of the next scheduled event.
         */
        return next_event;
    }
    /**
     * No-op. AsyncStateMachine doesn't need to initialize any slate tokens.
     *
     * @param builder The slate where to store this state machine's elements.
     *
     * @return True on success.
     */
    bool AsyncStateMachine::init_storage_state_machine(SlateBuilder builder)
    {
        return true;
    }
    /**
     * Finalize the initialization of the state machine. This is called after
     * common initialization is done to manage any steps specific to this type
     * of state machine. After this is called the state machine should be ready
     * for use.
     *
     * @return True on success.
     */
    bool AsyncStateMachine::init_finalize()
    {
        /*
         * Validate the graph formed by the states is safe for dispatching. We
         * don't consider single-cycles states dangerous because a cycle of
         * those would still allow external commands in, and would still request
         * a future dispatch time.
         */
        FswAbortIfNot(validate_graph(true /*allow_single_cycle*/), false);
        return true;
    }
    /**
     * Constructor.
     *
     * @param _cmd_table Reference to the CommandTable. Used to look up
     * vehicle commands.
     * @param _transition_period The period at which to check for state
     * transitions and dispatch event sequences and control tasks.
     * @param _local_verbosity   Affects what dbverbose() level is required
     * before debug messages are printed.
     */
    SyncStateMachine::SyncStateMachine(const CommandTable &_cmd_table,
                                       const nano_t _transition_period,
                                       const int _local_verbosity)
        : StateMachine(_cmd_table, _transition_period, _local_verbosity),
          last_pending_cmd_index_tok()
    {}
    /**
     * Destructor.
     */
    SyncStateMachine::~SyncStateMachine() {}
    /**
     * Dispatch any scheduled events.
     *
     * @param control_time The current time.
     *
     * @return We always return nano_t_max since this is designed to run in a
     * synchronous loop.
     */
    nano_t SyncStateMachine::dispatch(nano_t control_time) RUNTIME
    {
        const nano_t next_event = nano_t_max;
        /*
         * First we dispatch any pre-transition control tasks.
         */
        dispatch_pre_tasks(control_time);
        /*
         * See if we are completing a transition that started on the previous
         * dispatch (caused by a post task or timeout injecting a command).
         */
        const bool partial_transition =
            slate[last_pending_cmd_index_tok] != noop_cmd_index;
        FswAssert(!partial_transition ||
                  (partial_transition && (slate[last_pending_cmd_index_tok] ==
                                          slate[pending_cmd_index_tok])));
        /*
         * Check for pending state transitions. This might be a command from a
         * pre task, a partial transition from last cycle, or a command that
         * came in after the last cycle's dispatch.
         */
        bool transition_this_cycle = false;
        if (slate[pending_cmd_index_tok] != noop_cmd_index)
        {
            /*
             * If the pending command was part of a partial transition then we
             * already transitioned the pre tasks and ran the time-zero event
             * sequences during the last cycle so we now need to transition the
             * post tasks in the context of the new state. We don't do it inside
             * handle_pending_cmd(), though, because we want to transition the
             * post tasks after the current state has been set and the new state
             * signal has been emitted. If we didn't do this the order would be
             * different for pre task driven transitions and partial
             * transitions.
             *
             * If the pending command was not part of a partial transition then
             * we have not yet transitioned the pre tasks nor run the time-zero
             * event sequence, so we need to transition the pre tasks and then
             * prep the event sequence for its first dispatch. We won't yet
             * transition the post tasks though because we actually want to
             * dispatch the event sequence first in this case.
             */
            const bool transition_pre = !partial_transition;
            const bool start_seq = !partial_transition;
            FswIfNot(handle_pending_cmd(control_time, transition_pre,
                                        false /*see comment above*/,
                                        start_seq));
            transition_this_cycle = true;
        }
        /*
         * At this point there should be no pending commands teed up.
         */
        FswAssert(slate[pending_cmd_index_tok] == noop_cmd_index);
        FswAssert(slate[nominal_transition_time_tok] == nano_t_max);
        /*
         * Get the current state, which might have just gotten transitioned to.
         */
        FswAbortOutsideRange((uint)slate[cur_state_tok], 0U, state.size(),
                             next_event);
        Handle<RUNTIME ControlState> cs = state[slate[cur_state_tok]];
        FswAbortIfNot(cs, next_event);
        /*
         * Now it's safe to transition the post tasks if this is a partial
         * transition because we ran the first events last cycle and changed
         * the state and signaled it this cycle. We do this before we dispatch
         * the events from this cycle, though, so everything is consistent.
         */
        if (transition_this_cycle && partial_transition)
        {
            FswIfNot(transition_post_tasks(control_time, cs));
        }
        /*
         * Dispatch the event sequence if there is one, and look for a timeout
         * if it has reached the end.
         */
        if (!cs->seqs.empty())
        {
            const nano_t t = cs->dispatch_seqs(control_time);
            /*
             * Save off the current time into the current sequence for
             * telemetry.
             */
            slate[event_sequence_time_tok] = cs->get_current_time() / dbillion;
            /*
             * If the event sequence timed out, try to inject a timeout command
             * into the state machine.
             */
            if (t == nano_t_max)
            {
                vc_cmd_response_t response = vc_cmd_reject;
                nano_t retry_at = nano_t_max;
                FswIfNot(
                    handle_cmd(control_time, timeout_cmd, response, retry_at));
                /*
                 * We'll look later to see if this or any of the post tasks
                 * injected a command. We don't need to do anything special
                 * with the response here.
                 */
            }
        }
        /*
         * If we transitioned during this cycle then it's time to transition the
         * post tasks for the first time in the context of the new state's
         * events since we just dispatched the time-zero stuff for the new event
         * sequence. If this was a partial transition we already did this
         * earlier on this cycle.
         */
        if (transition_this_cycle && !partial_transition)
        {
            FswIfNot(transition_post_tasks(control_time, cs));
        }
        /*
         * Dispatch the post tasks. Note that we'll always transition these on
         * the same cycle that we dispatch them. This is an important property
         * to preserve for the synchronous state machines because GNC requires
         * this to operate properly.
         */
        dispatch_post_tasks(control_time);
        /*
         * If there's a pending command from a post task or a timeout then we
         * will immediately transition the pre tasks and dispatch the time-zero
         * events from the sequence that follows. This is the magic that allows
         * us to pull in the next state's events "early" because we can't run
         * again in the synchronous flavor. This is also why we ban zero length
         * states in this flavor because we cannot handle another transition at
         * this point in the cycle.
         */
        if (slate[pending_cmd_index_tok] != noop_cmd_index)
        {
            const uint pending_index = slate[pending_cmd_index_tok];
            FswAbortOutsideRange(pending_index, 0U, cs->cmds.size(),
                                 next_event);
            uint new_state_num = cs->cmds[pending_index];
            if (new_state_num < state.size())
            {
                Handle<RUNTIME ControlState> ncs = state[new_state_num];
                FswAbortIfNot(ncs, next_event);
                /*
                 * Transition the pre tasks in the context of the current
                 * state's events, before we apply the events for the next
                 * state. Note that we have to still use the next state's
                 * configuration for the control tasks because that is what
                 * is being configured.
                 */
                FswIfNot(transition_pre_tasks(control_time, ncs));
                /*
                 * Go ahead and pick up the events from the start of the next
                 * state, even though we haven't fully completed the transition.
                 * If we didn't do this these would get squashed with the events
                 * at the next control period.
                 */
                if (!FswIfNot(ncs->start_seqs(control_time)))
                {
                    ncs->dispatch_seqs(control_time);
                    /*
                     * Leave the event sequence time telemetry alone since we
                     * haven't set any other externally-visible indicators of
                     * the new state. We'll set it in the normal spot above on
                     * the next control cycle.
                     */
                }
            }
            /*
             * Leave the pending command as-is so we can complete this partial
             * transition on the next dispatch. We can't do any more this cycle
             * while still keeping the guarantees for post tasks about always
             * transitioning in the same cycle as the dispatch, and always
             * dispatching once if there's a transition.
             */
        }
        /*
         * Save off the pending command (if any) so that we can tell if any
         * pending command on the next dispatch is the completion of a state
         * transition from this cycle or some new command that got injected
         * after this dispatch is done.
         */
        slate[last_pending_cmd_index_tok] = slate[pending_cmd_index_tok];
        return next_event;
    }
    /**
     * Initialize slate tokens specific to SyncStateMachine.
     *
     * @param builder The slate where to store this state machine's elements.
     *
     * @return True on success.
     */
    bool SyncStateMachine::init_storage_state_machine(SlateBuilder builder)
    {
        SlateBuilder sub = builder.sub_slate("sm_" + role);
        FswAbortIfNot(sub.create("last_pending_cmd_index", noop_cmd_index,
                                 shard_sync, last_pending_cmd_index_tok),
                      false);
        return true;
    }
    /**
     * Finalize the initialization of the state machine. This is called after
     * common initialization is done to manage any steps specific to this type
     * of state machine. After this is called the state machine should be ready
     * for use.
     *
     * @return True on success.
     */
    bool SyncStateMachine::init_finalize()
    {
        /*
         * Validate the graph formed by the states is safe for dispatching. We
         * consider single-cycles states dangerous because a cycle of those
         * would prevent any external commands from getting processed, as there
         * would always be a pending timeout command.
         */
        FswAbortIfNot(validate_graph(false /*allow_single_cycle*/), false);
        /*
         * Synchronous flavor machines require that all event sequences are
         * at least as long as the control period, otherwise it will break the
         * assumption that we can begin the zero-time events for the following
         * state at the end of the prior state without worrying about another
         * transition happening due to a timeout.
         */
        FswAssert("timeout" == timeout_cmd.get_name());
        const uint timeout_index = timeout_cmd.get_index();
        /*
         * The initial state (state 0) is processed at initialization time, it
         * is required for it to have 0ms duration. Other states' duration
         * should be at least transition_period.
         */
        for (uint state_idx = 1; state_idx < state.size(); state_idx++)
        {
            Handle<ControlState> cs = state[state_idx];
            FswAssert(cs);
            FswAssert(timeout_index < cs->cmds.size());
            if ((!cs->seqs.empty()) && (cs->duration < transition_period) &&
                (cs->cmds[timeout_index] < state.size()))
            {
                FswPrefix();
                dbnprintf(400,
                          ": Synchronous state machines require all "
                          "states to be at least one control period long, "
                          "or have timeout commands ignored. State '%s' does "
                          "not meet both of these conditions.\n",
                          lookup_state(state_idx).c_str());
                return false;
            }
        }
        return true;
    }
    /**
     * Constructor.
     *
     * Other than the force_state() method, the
     * SettableSyncStateMachine is identical to a SyncStateMachine,
     * so this is a passthrough to the parent constructor.
     *
     * @param _cmd_table Reference to the CommandTable. Used to look
     * up vehicle commands.
     * @param _transition_period The period at which to check for state
     * transitions.
     * @param _local_verbosity Affects what dbverbose() level is required
     * before debug messages are printed.
     */
    SettableSyncStateMachine::SettableSyncStateMachine(
        const CommandTable &_cmd_table, nano_t _transition_period,
        int _local_verbosity)
        : SyncStateMachine(_cmd_table, _transition_period, _local_verbosity)
    {}
    /**
     * Force the state machine to a particular state, clearing any commands,
     * backdating the transition time, restarting event sequences, and setting
     * the event sequence time as requested.
     *
     * Event sequences will be restarted, but not dispatched; they will
     * "catch up" the next time the state machine is dispatched.
     *
     * @param control_time         The current control time.
     * @param state_num            The state to force a transition to.
     * @param cur_event_seq_time   The current event sequence time.
     *
     * @return True on success.
     */
    bool
    SettableSyncStateMachine::force_state(nano_t control_time, uint state_num,
                                          double cur_event_seq_time) RUNTIME
    {
        /*
         * Clear any pending commands and the nominal_transition_time.
         */
        slate[pending_cmd_index_tok] = noop_cmd_index;
        slate[nominal_transition_time_tok] = nano_t_max;
        slate[last_pending_cmd_index_tok] = noop_cmd_index;
        /*
         * Force the current state, calculating the backdated transition time
         * based on cur_event_seq_time.
         */
        FswAbortIfNot(set_state(state_num, noop_cmd_index,
                                control_time - to_nano_t(cur_event_seq_time),
                                true,  /* transition_pre */
                                true,  /* transition_post */
                                true), /* start_seq */
                      false);
        /*
         * set_state() sets the event sequence time to 0.0, reset it here.
         */
        slate[event_sequence_time_tok] = cur_event_seq_time;
        return true;
    }
    /**
     * Parses the command and state from the input words.
     *
     * @param words The words to parse. Contains a command and new state.
     * @param cmd The command
     * @param new_state The new state
     *
     * @return True on success
     */
    bool SettableSyncStateMachine::parse_state_transitions(
        const str_v &words, vehicle_cmd_t &cmd, uint &new_state) const
    {
        FswAbortIfNeqInt(words.size(), 3, false);
        FswAbortIfNot(lookup_cmd(words[1], cmd), false);
        if (words[1].find("uncontrollable") != std::string::npos)
        {
            new_state = StateMachine::ignore_state;
        }
        else
        {
            FswAbortIfNot(lookup_state(words[2], new_state), false);
        }
        return true;
    }
} /* end namespace Drone */