/**
 * @author Nathan DeVries
 * @date   2019/05/17
 */
#ifndef DRONE_GNC_CONTROL_H
#define DRONE_GNC_CONTROL_H
#include "src/bullwinkle/all/SlateCombiner.h"
#include "src/bullwinkle/all/SlateMapper.h"
#include "src/bullwinkle/all/SlateSharer.h"
#include "src/flight/common/all/BasicControl.h"
#include "src/flight/sat/all/common/ColaBurnMonitor.h"
#include "src/flight/sat/all/common/calibration_status_t.enum.h"
#include "src/flight/sat/all/common/control/TriggeredEventSequenceHandler.h"
#include "src/flight/sat/all/common/control/per_vehicle_calibration_util.h"
#include "src/flight/sat/all/common/satellite_constants.h"
#include "src/gnc/all/GncController.h"
#include "src/gnc/all/StateRegistrySlateInterface.h"
namespace Drone
{
    class AlertManagerControl;
    class CommonCommandFilter;
    class ExternalCommandGncHandler;
    class ExternalCommandMultiSlateHandler;
    class ReflectionManager;
    class TimedCommandQueue;
    /**
     * The control class for a Drone Orbit and Collision Avoidance
     * GNC.
     */
    class DroneGncControl : public BasicControl
    {
    public:
        static constexpr nano_t satgnc_control_period = 5 * billion;
        DroneGncControl(const std::string &_state_registry_dump_file);
        ~DroneGncControl() = default;

    private:
        ControlNodeIdentity get_slate_sharing_identity() const override;
        bool create_initial_systems() override;
        bool create_shared_sender_systems() override;
        bool create_shared_receiver_systems() override;
        bool create_alert_system() override;
        bool populate_enums() override;
        bool
        create_synced_command_handlers(const std::string &dispatcher_name,
                                       ext_cmd_handler_v &handlers) override;
        bool create_synced_command_filter(
            Handle<ExternalCommandFilter> &cmd_filter) override;
        bool pre_finalize_slate() override;
        bool
        finalize_control_systems(SlateBuilder sudo_slate_control,
                                 const cycle_timer_m &cycle_timers) override;
        bool create_basic_synced_command_platform() override;
        bool finalize_basic_command_platform(
            SlateBuilder sudo_slate_control) override;
        void execute_synced() RUNTIME override;
        bool create_gnc_system();
        bool create_control_systems(
            cycle_timer_name_s &cycle_timer_requests) override;
        bool build_state_machine_post_transition() override;
        bool setup_conjunction_receiver();
        bool setup_low_velo_conjunction_receiver();
        bool setup_reference_trajectory_receiver();
        bool setup_space_weather_receiver();
        bool setup_remote_node_receiver(const std::string &component_name,
                                        const sharer_config_v &receiver_config,
                                        const std::string &subslate_name,
                                        const std::string &freshness_name,
                                        const nano_t cycle_delay = nano_t_max,
                                        const bool downselect = false);
        bool add_receiver_reflection(const std::string &subslate_name);
        bool handle_sm_commands() RUNTIME;
        calibration_status_t parse_calibration();
        calibration_status_t parse_permanent_failures(
            const std::vector<calibration_dir_t> &root_reldirs);
        calibration_status_t parse_suppressed_alerts(
            const std::vector<calibration_dir_t> &root_reldirs);
        /**
         * Holds all GNC state.
         */
        Handle<StateRegistry> state_registry;
        /**
         * Initializes all of our GNC algorithms.
         */
        Handle<GncComponentFactory> gnc_component_factory;
        /**
         * Handle to our GNC task.
         */
        Handle<GncController> gnc;
        /**
         * Slate receivers.
         */
        std::vector<Handle<SlateSharerReceiver>> sharer_receivers;
        /**
         * Contains a list of all of the slate combiners we need to service.
         */
        std::vector<Handle<SlateCombiner>> sharer_combiners;
        /**
         * Handle to a manager that will create and dispatch the state registry
         * interfaces conditionaly based on valid reference trajectory.
         */
        Handle<StateRegistrySlateInterfaceManager>
            sr_conditional_interface_manager;
        /**
         * Handle to a manager that will create and dispatch the state registry
         * interfaces we need.
         */
        Handle<StateRegistrySlateInterfaceManager> sr_interface_manager;
        /**
         * A SlateMapper for GncControl <=> DroneControl communications.
         */
        Handle<SlateMapper> satfc_comms_mapper;
        /**
         * A command spammer that continually attempts to send the last command
         * issued by the main satfc control process until it is accepted.
         */
        Handle<CommandSpammer> satfc_command_spammer;
        /**
         * Token for state machine command from the main control process.
         */
        ReadToken<INT32> satfc_cmd_tok;
        /**
         * Token for previous state machine command from the main control
         * process.
         */
        WriteToken<INT32> old_satfc_cmd_tok;
        /**
         * Token for reference trajectory sequence number.
         */
        ReadToken<INT64> reference_trajectory_sequence_number_tok;
        /**
         * Last received reference trajectory sequence number.
         */
        WriteToken<INT64>
            last_received_reference_trajectory_sequence_number_tok;
        /**
         * Whether we've ever dispatch the state machine.
         * We track this because we don't want to command the state machine
         * during the first command cycle.
         */
        WriteToken<bool> state_machine_ever_dispatched_tok;
        /**
         * GNC command handler.
         */
        Handle<ExternalCommandGncHandler> gnc_command_handler;
        /**
         * Multi command handler.
         */
        Handle<ExternalCommandMultiSlateHandler> multi_command_handler;
        /**
         * Timed command queue.
         */
        Handle<TimedCommandQueue> timed_command_queue;
        /**
         * Reflection Manager.
         */
        Handle<ReflectionManager> reflection_manager;
        /**
         * Command filter for the synced command inputs.
         */
        Handle<CommonCommandFilter> command_filter_synced;
        /**
         * Aggregates and transmits signaled alerts.
         */
        Handle<AlertManagerControl> alert_manager;
        /**
         * A token indicating the status of calibration parsing.
         */
        WriteToken<UINT8> calibration_status_tok;
        /**
         * Triggered Event Sequence handler for failed hardware sequences.
         */
        Handle<TriggeredEventSequenceHandler> failed_hardware_seq_handler;
        /**
         * A token indicating if dumping the state registry to a file is
         * enabled.
         */
        WriteToken<bool> state_registry_dumps_enabled_tok;
        /**
         * Monitor that listens for COLA burns (or maneuver_required states)
         * and dumps the state registry.
         */
        Handle<ColaBurnMonitor> cola_burn_monitor;
        /**
         * Path to dump state registry snapshot. If empty, dumps will be
         * disabled.
         */
        const std::string state_registry_dump_file;
    };
} /* namespace Drone */
#endif /* DRONE_GNC_CONTROL_H */