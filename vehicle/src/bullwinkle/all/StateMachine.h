/**
 * @author Chris Sloan
 * @date   11/11/04
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H
#include "src/bullwinkle/all/CommandTable.h"
#include "src/bullwinkle/all/ControlState.h"
#include "src/bullwinkle/all/ControlTask.h"
#include "src/bullwinkle/all/EnumRegistry.h"
#include "src/bullwinkle/all/EventLoop.h"
#include "src/bullwinkle/all/EventSequence.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/config_file.h"
#include "src/bullwinkle/all/enum/SymbolTable.h"
#include "src/bullwinkle/all/runtime.h"
#include "src/bullwinkle/all/vehicle_cmd.h"
#include <set>
#include <vector>
namespace Drone
{
    /**
     * An enumeration of responses from handle_cmd to indicate how a
     * command was handled.
     */
    enum vc_cmd_response_t
    {
        /**
         * The command was accepted and has been or will be handled.
         */
        vc_cmd_accept,
        /**
         * The command can not be accepted at this time.  Call again
         * later.  Usually a command is deferred because there is
         * already a pending command and a new command can not be
         * accepted until that command has been handled completely.
         */
        vc_cmd_defer,
        /**
         * This command was rejected because it is not expected nor
         * defined in the current mode.
         *
         * Generally the caller should not call back with this
         * command, but should discard it instead.
         *
         * @note Commands which are expected and marked ignore return
         * vc_cmd_accept, not vc_cmd_reject.
         */
        vc_cmd_reject,
    };
    /**
     * State machine base class. You can't instantiate this directly,
     * choose SyncStateMachine or AsyncStateMachine depending on your
     * requirements.
     *
     * In general, state machines manage a set of states with associated
     * event sequences and control tasks. See \ref state_machine for more
     * details.
     */
    class StateMachine : public EventSource, public SignalHandler
    {
    public:
        StateMachine(const CommandTable &_cmd_table, nano_t _transition_period,
                     int _local_verbosity = default_verbosity,
                     nano_t _initial_control_time = 0LL);
        virtual ~StateMachine();
        /*
         * The "external interface."
         *
         * These functions are called by code outside the StateMachine which are
         * not ControlTasks.
         *
         * init_storage() needs to be called first.
         * init() should be called once the slate is built.
         */
        bool init_storage(const Configs &configs, SlateBuilder builder,
                          const std::string &_role);
        bool init(SlateBuilder builder, const Configs &configs,
                  const EnumRegistry &enum_registry);
        std::string get_state() const;
        static uint hash_state(const std::string &state);
        uint get_state_num() const;
        bool add_ctask(const std::string ctask_name, Handle<ControlTask> ctask);
        bool pre_transition_ctasks_added();
        bool set_state_machine_name(const std::string &name);
        const CommandTable &get_cmd_table() const;
        const SymbolTable &get_state_sym_table() const;
        const std::string &get_control_prefix() const;
        /**
         * Constant used to indicate that a command has no defined
         * action in a particular state.
         */
        static const uint undefined_state;
        /**
         * Constant used to indicate that a command is to be ignored
         * in a particular state.
         */
        static const uint ignore_state;
        /**
         * The default verbosity of StateMachine. Used to control what is
         * printed by a StateMachine. If no verbosity is specified in the
         * constructor, this verbosity is used.
         */
        static const int default_verbosity;
        typedef RUNTIME_SIGNAL<bool, const std::string &> state_sig_t;
        /**
         * Used by external modules which need to be informed of state
         * changes.
         */
        state_sig_t state_sig;
        /*
         * The "internal interface."
         *
         * The following functions are intended to be called by ControlTasks and
         * related code, not by outside code.
         */
        bool is_cmd_allowed(vehicle_cmd_t cmd) const;
        bool handle_cmd(nano_t control_time, vehicle_cmd_t cmd,
                        vc_cmd_response_t &response, nano_t &retry_at) RUNTIME;
        bool lookup_cmd(const std::string &name, vehicle_cmd_t &cmd) const;
        bool lookup_cmd(const uint index, vehicle_cmd_t &cmd) const;
        bool has_state(const std::string &name) const;
        bool has_cmd(vehicle_cmd_t cmd) const;

    protected:
        bool set_initial_state();
        bool lookup_state(const std::string &name, uint &s) const;
        std::string lookup_state(uint s) const;
        bool lookup_ctask(const std::string &name, uint &ctask) const;
        std::string lookup_ctask(uint ctask) const;
        bool lookup_cstate(const std::string &name,
                           ctask_state_t &cstate) const;
        std::string lookup_cstate(ctask_state_t cstate) const;
        bool parse_multi_seq_devices(SlateBuilder &builder,
                                     const Configs &configs,
                                     const std::string &_role,
                                     str_str_s_m &multi_seq_devices);
        virtual bool parse_state_transitions(const str_v &words,
                                             vehicle_cmd_t &cmd,
                                             uint &new_state) const;
        bool initialize_state_from_file(const Configs &configs,
                                        SlateBuilder builder, uint state_num);
        bool finalize_state_from_file(const Configs &configs, uint state_num,
                                      const str_str_s_m &multi_seq_devices);
        bool validate_state(const ControlState &cs, const uint state_idx) const;
        /**
         * Enumeration of the possible states of a node in the graph of
         * state machine states while do a \ref validate_graph call.
         */
        enum color
        {
            unexplored,  //< Node has not been explored.
            in_progress, //< Node is currently being explored.
            explored,    //< Node has been fully explored.
        };
        bool validate_graph(bool allow_single_cycle) const;
        bool search_for_cycle(const std::set<uint> &dangerous_states,
                              std::vector<color> &colors, bool &cycle_detected,
                              uint state_idx) const;
        bool handle_pending_cmd(nano_t transition_time, bool transition_pre,
                                bool transition_post, bool start_seq) RUNTIME;
        bool handle_pending_cmd_aux(nano_t transition_time, bool transition_pre,
                                    bool transition_post,
                                    bool start_seq) RUNTIME;
        bool set_state(uint state_num, uint cmd_index, nano_t transition_time,
                       bool transition_pre, bool transition_post,
                       bool start_seq) RUNTIME;
        bool transition_task(nano_t transition_time, uint task,
                             Handle<RUNTIME ControlState> cs) RUNTIME;
        bool transition_pre_tasks(nano_t transition_time,
                                  Handle<RUNTIME ControlState> cs) RUNTIME;
        bool transition_post_tasks(nano_t transition_time,
                                   Handle<RUNTIME ControlState> cs) RUNTIME;
        bool set_nominal_transition_time(nano_t control_time) RUNTIME;
        nano_t dispatch_pre_tasks(nano_t control_time) RUNTIME;
        nano_t dispatch_post_tasks(nano_t control_time) RUNTIME;
        /*
         * The "flavor-specific" interface.
         *
         * Async and sync subclasses implement their specific behaviors by
         * defining these methods.
         */
        virtual bool init_storage_state_machine(SlateBuilder builder) = 0;
        virtual bool init_finalize() = 0;

    public:
        virtual nano_t dispatch(nano_t control_time) RUNTIME = 0;

    protected:
        /**
         * Set to true if init() completed successfully.
         */
        bool is_init;
        /**
         * Set to true if init_storage() completed successfully.
         */
        bool is_init_storage;
        /**
         * Time to use to initialize the initial event sequence.
         */
        const nano_t initial_control_time;
        /**
         * The slate run-time object.
         */
        INFRASTRUCTURE(Slate) slate;
        /**
         * Name of this state machine, for console printing.
         */
        std::string state_machine_name;
        /**
         * The role of this computer (fc, ec, or rio, etc).
         *
         * During initialization, this is used to find configuration
         * files and to decide how to configure the state machine.
         */
        std::string role;
        /**
         * Subtree path to our slate.
         */
        std::string control_prefix;
        /**
         * The list of valid StateMachine commands.
         */
        const CommandTable &cmd_table;
        /*
         * List of state machine state names.
         */
        str_v state_v;
        /**
         * A symbol table mapping between state numbers and state
         * names.
         */
        SymbolTable state_sym;
        /**
         * A symbol table mapping between control task numbers and
         * control task names. This map is dynamically constructed as
         * control tasks are added to the state machine.
         */
        SymbolTable ctask_sym;
        /**
         * A vector mapping task number to ControlTask.
         */
        std::vector<Handle<ControlTask>> task;
        /**
         * A vector mapping state number to ControlState.
         */
        std::vector<Handle<ControlState>> state;
        /**
         * The special timeout command.
         */
        vehicle_cmd_t timeout_cmd;
        /**
         * The current pending command or noop_cmd_index if no commands are
         * currently pending.
         */
        WriteToken<uint> pending_cmd_index_tok;
        /**
         * The period at which to check for state transitions.
         */
        const nano_t transition_period;
        /**
         * The nominal time of the pending state transition.
         *
         * If no state transition is pending, then its value will be
         * nano_t_max.
         */
        WriteToken<nano_t> nominal_transition_time_tok;
        /**
         * This is true once pre_transition_ctasks_added() has been
         * called, so we can catch errors if it is called again.
         */
        bool pre_transition_tasks_added;
        /**
         * The number of control tasks that should be dispatched
         * prior to the the state transition.
         */
        uint num_pre_transition_ctasks;
        /**
         * Affects what dbverbose() level is required before debug
         * messages are printed.
         */
        const int local_verbosity;
        /**
         * Device ID for event sequence time.
         */
        WriteToken<double> event_sequence_time_tok;
        /**
         * The current state machine state.
         */
        WriteToken<INT32> cur_state_tok;
        /**
         * The previous state machine state.
         */
        WriteToken<INT32> old_state_tok;
        /**
         * The state machine state before the previous state.
         */
        WriteToken<INT32> older_state_tok;
    };
    /**
     * Asynchronous state machines expect to be run in an event loop
     * (or equivalent) that is free to dispatch as often as is necessary
     * based on the lowest "next" time of its constituent tasks. Zero
     * length event sequences are supported, provided they don't form a
     * cycle.
     *
     * See \ref async_state_machines for more details.
     */
    class AsyncStateMachine : public StateMachine
    {
    public:
        AsyncStateMachine(const CommandTable &_cmd_table,
                          nano_t _transition_period,
                          int _local_verbosity = default_verbosity,
                          nano_t _initial_control_time = 0LL);
        virtual ~AsyncStateMachine();
        virtual nano_t dispatch(nano_t control_time) RUNTIME;

    protected:
        virtual bool init_storage_state_machine(SlateBuilder builder);
        virtual bool init_finalize();
    };
    /**
     * Synchronous state machines expect to be run in an event loop (or
     * equivalent) that only dispatches its constituent tasks once per cycle,
     * even if their lowest "next" time is less than a control period. Special
     * provisions are made to run the time-zero events from a new state at the
     * end of the cycle of the previous state. To allow for this, all states
     * must be at least one control period long (i.e., contain at least one
     * event sequence whose duration is one control period or longer) or ignore
     * the timeout command.
     *
     * See \ref sync_state_machines for more details.
     */
    class SyncStateMachine : public StateMachine
    {
    public:
        SyncStateMachine(const CommandTable &_cmd_table,
                         nano_t _transition_period,
                         int _local_verbosity = default_verbosity);
        virtual ~SyncStateMachine();
        virtual nano_t dispatch(nano_t control_time) RUNTIME;

    protected:
        virtual bool init_storage_state_machine(SlateBuilder builder);
        virtual bool init_finalize();
        /**
         * The pending command that was pending when we were last dispatched.
         * This lets us figure out if we are completing a transition that began
         * on the previous cycle, or if something else injected a command after
         * we were dispatched.
         */
        WriteToken<uint> last_pending_cmd_index_tok;
    };
    /**
     * A subclass of SyncStateMachine that allows itself to be "forced" into a
     * new state, which clears any pending commands before transitioning to the
     * new state.
     */
    class SettableSyncStateMachine : public SyncStateMachine
    {
    public:
        SettableSyncStateMachine(const CommandTable &_cmd_table,
                                 nano_t _transition_period,
                                 int _local_verbosity = default_verbosity);
        bool force_state(nano_t control_time, uint state_num,
                         double cur_event_seq_time) RUNTIME;
        bool parse_state_transitions(const str_v &words, vehicle_cmd_t &cmd,
                                     uint &new_state) const override;
    };
} /* end namespace Drone */
/**
 * @page state_machine StateMachine State Machine
 *
 * The StateMachine state machine is a central component of
 * vehicle_control.  The state machine controls which sequences are
 * run, which commands are responded to (and how), and which
 * ControlTasks are active in a given state.
 *
 * The state machine is described by the configuration files.  The
 * master files (fc.master or ec.master) list all of the ControlStates
 * (or just "states") in the state machine.  For each ControlState
 * there is a file named "<state>.state" which describes it.  For
 * example, the safe state is described by the "safe.state" file.
 * Within each state file is a set of directives.
 *
 * A directive of the form "seq file1.event [file2.event] [file3.event]..."
 * specifies which event sequences to run during that state. The sequence
 * directive may have one or more files specified, but may not be empty. The
 * sequences are started as soon as the state is transitioned to and continues
 * only while the state remains active. If StateMachine leaves the state, no
 * further events from the sequences will be executed.  When all of the
 * sequences in the state are completed, it generates a "timeout" command.  If
 * an action is defined for this command, the state machine will carry out that
 * action (presumably transitioning to a new state). If no action is defined it
 * will be implicitly and silently ignored. This means that states with no
 * "timeout" action will keep running even after their sequences are complete.
 * If any action including "timeout" points back to the same state, the
 * sequences will be restarted.  If the "seq" directive is absent, no sequences
 * will be run in that state.
 *
 * When multiple event sequences are running, at each control cycle, all the
 * events that will happen at that cycle from every sequence happen. No
 * guarantees are made about the order in which these events happen. This order
 * must not be relied on for proper operation.
 *
 * The following rules are enforced when multiple event sequences are present in
 * a single control state. Event sequences in the same state must:
 * - Have unique names.
 * - Not command the same devices or state registry entries.
 * - Start at the same time (note that if a sequence's first event is at some
 *   positive time, no matter the time, the sequence is regarded as starting
 *   at zero time). This can be done with autosequence_begins.
 * - If the timeout command is not ignored, then all event sequences must end
 *   at the same time. This can be done with autosequence_ends.
 *
 * The same rules apply to all event sequences in a control state, even when
 * updating an event sequence in a control state.
 *
 * A directive of the form "cmd <command> <action>" (e.g. "cmd abort
 * safe") means that if the specified command is received, the state
 * machine should take the supplied action.  Actions are usually state
 * names meaning that the state machine should transition to that
 * state, but they can also be "ignore" if that command is to be
 * ignored.  A command which has no action will generate a diagnostic
 * warning on the console, but will be otherwise ignored, so all
 * messages which the state machine can receive should either result
 * in a state change action or an ignore action.  The exception is the
 * "timeout" command which does not require an explicit ignore
 * action as mentioned above.
 *
 * A directive of the form "task <task_name> <task_state> <args...>"
 * (e.g. "task ec cmd abort") specifies the state of a ControlTask (or
 * "task") when the ControlState is transitioned to.  The ControlTask
 * names are defined in ctask_t.enum.  The ControlTask states are
 * defined in ctask_state_t.enum and include "on," "off," "cont," and
 * "cmd."
 *
 * The "on" state will activate the task if it is not active.  If
 * additional arguments are present, they will be passed to
 * Drone::ControlTask::start(nano_t,const std::vector<std::string>&).
 * This allows the state machine to pass task-specific arguments
 * to the task.
 *
 * The "cmd" state will send a command (the additional arguments) to
 * an active task.  It is an error to send such a command to an
 * inactive task.  The command will be interpreted in a task-specific
 * way.
 *
 * The "cont" state indicates that an active task should continue
 * running in this mode.  No commands are sent.
 *
 * The "off" state (which is the default if a task is not mentioned in
 * a given state) will deactivate (or keep a task deactivated) when
 * StateMachine transitions to the ControlState.  Since "off" is the
 * default, any unmentioned tasks will be deactivated.  The system
 * does not currently force the use of the "off" state to turn off a
 * task, but it can be used to document that the task is being turned
 * off at that state transition.
 *
 * In order to provide an opportunity to initialize devices, Event sequences
 * specified for the first state (i.e. the initial state) are processed
 * immediately during the initialization of the state machine. Commands
 * specified in the initial state are processed when the state machine is being
 * dispatched.
 *
 * Because the time between initialization and dispatching of the state machine
 * is arbitrary, the initial state is restricted:
 * - Its duration needs to be 0ms: it has zero-length event sequences or no
     sequences at all.
 * - It does not start, command, continue, or stop any control tasks.
 * - If a timeout command is specified, no other command can be (since there
 *   wouldn't be an opportunity to process them).
 * - It is not possible to command back to the initial state.
 *
 * @section sync_state_machines Synchronous State Machines
 *
 * Synchronous state machines expect to be run in an event loop (or equivalent)
 * that only dispatches its constituent tasks once per cycle, even if their
 * lowest "next" time is less than a control period. Special provisions are made
 * to run the time-zero events from a new state at the end of the cycle of the
 * previous state. To allow for this, all event sequences in a control state
 * must be at least one control period long or ignore the timeout command.
 *
 * Some guarantees:
 *
 * 1. All control task dispatch() functions will be called exactly once per
 *    control cycle.
 * 2. The time passed into the ControlTask dispatch functions will always be the
 *    current control cycle time.
 * 3. All task_start/stop/cmd functions will be called, at most, once per cycle.
 * 4. The time passed into the task_start/stop/cmd functions will always be the
 *    current control cycle time.
 * 5. If task_start/stop/cmd functions are called, all post-transition tasks
 *    will have have their task_start/stop/cmd function called before their
 *    dispatch function is called within a given control cycle, and all
 *    pre-transition tasks will have their task_start/stop/cmd functions called
 *    after their dispatch functions for a given control cycle. Pre-transition
 *    tasks will see these calls before the t=0 events for the next state are
 *    applied. Post-transition tasks will see these calls after the t=0 events
 *    for the next state are applied.
 * 6. All pre-transition tasks will have their dispatch functions called between
 *    each time slice of the event sequence. In other words, if two events in an
 *    event sequence are separated by one control cycle's worth of time, there
 *    will be exactly one call to a pre-transition's dispatch function in
 *    between them.
 * 7. If two commands in the same event sequence file are separated by x
 *    seconds, they will always take place x real seconds apart. If a series of
 *    event sequences run back-to-back with timeouts, then the total duration of
 *    this series will equal the sum of the durations of each component event
 *    sequence (no gaps will be introduced and no intervals will be squashed).
 * 8. All state transitions and calls to control task functions will happen
 *    during the SyncStateMachine's dispatch function.
 * 9. There will be at most one state transition per control cycle.
 * 10. Within those limitations, all commands will be handled as quickly as
 *     possible.
 *
 * Some nuances:
 *
 * 1. For post-transition tasks, the ordering of event sequence dispatches and
 *    control task dispatches may change. Normally, post-transition tasks are
 *    dispatched after event sequences are dispatched. When processing a timeout
 *    or a command from a post-transition task, there's not an opportunity to
 *    dispatch the post-transition tasks right after the time-zero events are
 *    dispatched, so for this case the post-transition tasks will be dispatched
 *    after time-zero and the first cycle's events have been dispatched.
 * 2. Pre-transition tasks may have their task_start/stop/cmd functions called
 *    the control cycle before the state machine actually changes state.
 * 3. Event sequences may start to run the cycle before they are 'announced' -
 *    that is, an event sequence for state B may already be running a cycle
 *    before the "state B starting" signal is emitted and before telemetry is
 *    updated to reflect the switch to state B.
 * 4. For all tasks, calling any function that queries the state machine for its
 *    current state, or that indirectly relies on the current state (whether
 *    derived from devices or relying on the emitted signal) may produce
 *    inconsistent results.
 * 5. Two different event sequences may run on the same control cycle during a
 *    transition (but the new event sequence will always run after the old event
 *    sequence).
 *
 * @section async_state_machines Asynchronous State Machines
 *
 * Asynchronous state machines expect to be run in an event loop
 * (or equivalent) that is free to dispatch as often as is necessary
 * based on the lowest "next" time of its constituent tasks. Zero
 * length event sequences are supported, provided they don't form a
 * cycle.
 *
 * Some nuances:
 *
 * 1. Pre-transition and post-transition control tasks will get their
 *    stop/start/cmd calls in the context of the previous state when
 *    transitions occur.
 * 2. Control tasks will see back-dated times in their start/stop/cmd
 *    calls when transitions occur because of a timeout on the previous
 *    cycle or a command from a post-transition task.
 *
 * A common pattern when embedding asynchronous state machines is to
 * loop and call dispatch() on the state machine until it returns a
 * next time in the future, indicating that it has completed everything
 * it needed to do for the current time instant. In order for this to
 * terminate and be safe you need to analyze the following items, which
 * all affect the next time that an asynchronous state machine reports:
 *
 * 1. The minimum wakeup time returned by pre-transition tasks.
 * 2. nano_t_min if a state transition took place.
 * 3. The next wakeup time returned by the event sequence.
 * 4. The minimum wakeup time returned by post-transition tasks.
 * 5. The nominal transition time, which will be set in the past if a command
 *    is pending.
 *
 * The state machine will ensure that you don't have any cycles in the state
 * graph that would get stuck with a series of zero-length states and timeouts.
 *
 * @section state_machine_rules State Machine Rules
 *
 * There are a number of rules or invariants which are important to
 * maintain in the design or modification of the vehicle state
 * machines.  The following list tries to capture as many as
 * possible.  Some are automatically checked while the state machine
 * configuration is being parsed.
 *
 * - The abort* states (abort_1, abort_2, ...) only transition to
 *   other abort* states or the safe state.
 *
 * - The safe* states except for safe_idle (safe, safe_wait*, ...)
 *   only transition to other safe* modes.
 *
 * - If the action on disconnect or abort is not ignore it is always
 *   abort* or safe.
 *
 * - The abort command can only be ignored in abort* or safe* states.
 *
 * - The state machine will force entry to safe_idle, manual, and
 *   auto_idle to be synchronized across computers.  Additionally, if
 *   one machine loses communications or leaves one of these states, the
 *   other will do so as well (eg. by the watchdog or other mechanism).
 *
 * - If communications are functional, the following states are
 *   reached (more or less) simultaneously by both computers: safe_idle,
 *   auto_idle, and manual.
 *
 * - If the flight computer goes to abort* or safe, it must send
 *   aborts to synchronize with the EC, not disconnects.
 *
 * - A state with a task "off" can only transition to states where the
 *   task is either "on" or "off" (or unmentioned and therefore off by
 *   default).  It cannot transition to a state where that task is
 *   "cont" or "cmd."
 */
#endif /* STATEMACHINE_H */