/**
 * @author Stefan Moluf
 * @date   2013/02/07
 */
#ifndef BASIC_CONTROL_H
#define BASIC_CONTROL_H
#include "src/bullwinkle/all/AdcPowersaver.h"
#include "src/bullwinkle/all/AdcScaler.h"
#include "src/bullwinkle/all/AlarmEventSequenceHandler.h"
#include "src/bullwinkle/all/AlertProvider.h"
#include "src/bullwinkle/all/CommandSpammer.h"
#include "src/bullwinkle/all/Configs.h"
#include "src/bullwinkle/all/DeviceTelemetryFactory.h"
#include "src/bullwinkle/all/EnumRegistry.h"
#include "src/bullwinkle/all/EnumeratedBitmapManager.h"
#include "src/bullwinkle/all/ExternalCommandDispatcher.h"
#include "src/bullwinkle/all/ExternalCommandSlateHandler.h"
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/SlateAlarmTask.h"
#include "src/bullwinkle/all/SlateCombiner.h"
#include "src/bullwinkle/all/SlateSharer.h"
#include "src/bullwinkle/all/SlateSharerManager.h"
#include "src/bullwinkle/all/StateMachine.h"
#include "src/bullwinkle/all/TelemetryRelay.h"
#include "src/bullwinkle/all/ft/FtChannelManager.h"
#include "src/bullwinkle/all/io/DgramChannel.h"
#include "src/bullwinkle/all/runtime.h"
#include "src/flight/common/all/ControlInterface.h"
#include "src/flight/common/all/TimestampSynchronizer.h"
#include "src/flight/common/all/TripleString.h"
#include <utility>
#include <vector>
namespace Drone
{
    /**
     * A skeleton for building control binaries, including:
     *
     *  - State and event control with StateMachine.
     *  - Alarm detection and response with SlateAlarms.
     *  - Ground commanding with BwpRequestDispatcher (talks to CommandProxy).
     *
     * BasicControl has pluggable points to insert vehicle-specific system
     * initialization, and all non-utility functions are virtualized so that one
     * can blow away whole sections if necessary.
     *
     * The initialization phases are as follows:
     *
     *  1. The string-sharing mechanisms are created.
     *  2. The synchronized control systems are created. This includes the
     *     state machine, alarms and the commanding platform.
     *
     */
    class BasicControl : public ControlInterface
    {
    public:
        BasicControl(
            const ExternalCommandDispatcher::config_params_t &_ext_cmd_params,
            const std::string &_telem_config_key);
        virtual ~BasicControl();
        bool init_basic_identity(const nano_t _control_period,
                                 const Configs &_configs,
                                 const std::string &process_name,
                                 const ControlNodeIdentity &_ident,
                                 SlateBuilder _slate_control,
                                 FtNodeConfigList &node_configs) override;
        bool
        init_pre_slate_build(Handle<Keychain> _keychain,
                             Handle<FtChannelManager> _channel_manager,
                             const adc_board_v &_adc_boards_info,
                             Handle<EnumRegistry> _enum_registry,
                             cycle_timer_name_s &cycle_timer_requests) override;
        bool init_post_slate_build(SlateBuilder sudo_slate_control,
                                   const cycle_timer_m &cycle_timers) override;
        bool get_scaling(scale_info_v &scaling) const override;
        void start_cycle(const nano_t control_time) override;
        void dispatch_synced() RUNTIME override;

    protected:
        virtual void execute_synced() RUNTIME = 0;
        virtual void execute_send_slate_sharing() RUNTIME;
        virtual ControlNodeIdentity get_slate_sharing_identity() const;
        /*
         * Major initialization phases. Do not override these unless
         * absolutely necessary.
         */
        virtual bool create_basic_initial_platform();
        virtual bool create_basic_platform();
        virtual bool create_basic_shared_receiver_platform();
        virtual bool create_basic_control_platform();
        virtual bool create_basic_synced_command_platform();
        virtual bool finalize_enums();
        virtual bool pre_finalize_slate();
        virtual bool post_finalize_slate(SlateBuilder sudo_slate_control);
        virtual bool finalize_legacy_systems(SlateBuilder sudo_slate_control);
        virtual bool
        finalize_basic_command_platform(SlateBuilder sudo_slate_control);
        virtual bool build_state_machine();
        virtual bool basic_runtime_checks();
        /*
         * Systems creation phases. Use these for vehicle-specific code.
         */
        virtual bool create_initial_systems();
        virtual bool create_shared_receiver_systems();
        virtual bool create_slate_sender_creators();
        virtual bool create_alert_system();
        virtual bool
        create_control_systems(cycle_timer_name_s &cycle_timer_requests);
        virtual bool create_shared_sender_systems();
        virtual bool create_synced_command_deframer(
            Handle<ExternalCommandDeframer> &deframer);
        virtual bool
        create_synced_command_filter(Handle<ExternalCommandFilter> &cmd_filter);
        virtual bool
        create_synced_command_handlers(const std::string &dispatcher_name,
                                       ext_cmd_handler_v &handlers);
        virtual bool
        finalize_control_systems(SlateBuilder sudo_slate_control,
                                 const cycle_timer_m &cycle_timers);
        virtual bool create_legacy_systems();
        virtual bool populate_enums();
        virtual bool runtime_checks();
        /*
         * Dispatch ordering registration.
         */
        virtual bool build_state_machine_pre_transition();
        virtual bool build_state_machine_post_transition();
        /*
         * Component initialization functions.
         */
        virtual bool
        create_ravenscript(cycle_timer_name_s &cycle_timer_requests);
        virtual bool create_alarm_system();
        virtual bool finalize_alarm_system();
        /*
         * Class initialization convenience functions.
         */
        bool create_slate_sender(SlateBuilder source,
                                 const sharer_config_v &config_list,
                                 const std::string &sender_name,
                                 const std::string &output_channel,
                                 const bool is_triple_string,
                                 const sharer_create_or_bind_t create_or_bind,
                                 Handle<SlateSharerSender> &sender);
        bool create_slate_receiver(SlateBuilder destination,
                                   const sharer_config_v &config_list,
                                   const std::string &input_channel,
                                   const nano_t cycle_delay,
                                   Handle<SlateSharerReceiver> &receiver);
        bool create_adc_scaler(SlateBuilder raw, SlateBuilder scaled,
                               SlateBuilder median,
                               const bank_select_type_t ad_scaling_set,
                               Handle<AdcScaler> &scaler);
        bool create_adc_unscaler(SlateBuilder scaled, SlateBuilder raw,
                                 Handle<AdcUnscaler> &unscaler);
        /*
         * Event handlers.
         */
        bool handle_alarm(RUNTIME SlateAlarm &alarm,
                          Handle<StateMachine> sm) RUNTIME;
        void reset_counters() RUNTIME;
        /**
         * The set of parameters used to configure the control (synchronized)
         * ExternalCommandDispatcher.
         */
        const ExternalCommandDispatcher::config_params_t ext_cmd_params;
        /**
         * Name of the telemetry config file used to create control devices for
         * the telemetry system.
         */
        const std::string telem_config_key;
        /**
         * The Clock object for this control code.
         */
        ControlClock clock;
        /**
         * The config-file finding object.
         */
        Configs configs;
        /**
         * True when initialized.
         */
        bool is_init;
        /**
         * The period at which we control.
         */
        nano_t control_period;
        /**
         * Our identity.
         */
        ControlNodeIdentity ident;
        /**
         * The slate for data slate-shared from each string.
         */
        TripleString<SlateBuilder> slate_shared;
        /**
         * The slate for our unified pan-string control data.
         */
        SlateBuilder slate_control;
        /**
         * The run-time Slate.
         */
        INFRASTRUCTURE(Slate) slate;
        /**
         * The keychain for signing messages.
         */
        Handle<Keychain> keychain;
        /**
         * The channel manager, which handles all fault-tolerant input
         * and output data.
         */
        Handle<FtChannelManager> channel_manager;
        /**
         * The Slate sharer manager.
         */
        Handle<SlateSharerManagerTripleString> slate_sharer_manager;
        /**
         * Scaling information for Slate sharers hosted by SlateSharerManager.
         */
        scale_info_v slate_sharer_scaling;
        /**
         * Alarm inhibit. Setting this causes handle_alarm() to ignore alarms at
         * or below that level.
         */
        ReadToken<int> alarm_inhibit_tok;
        /**
         * Flag set when all counter statistics should be reset.
         */
        WriteToken<bool> reset_counters_tok;
        /**
         * The parsed-out hardware configuration.
         */
        adc_board_v adc_boards_info;
        /**
         * The valve powersaver event source. This switches full power
         * valves into powersave mode after a configurable duty cycle.
         */
        Handle<AdcPowersaver> adc_powersaver;
        /**
         * The hardware scaler for the 'a' string shared space.
         */
        Handle<AdcScaler> adc_scaler_shared_a;
        /**
         * The hardware scaler for the 'b' string shared space.
         */
        Handle<AdcScaler> adc_scaler_shared_b;
        /**
         * The hardware scaler for the 'c' string shared space.
         */
        Handle<AdcScaler> adc_scaler_shared_c;
        /**
         * The hardware scaler for the control space.
         */
        Handle<AdcScaler> adc_scaler_control;
        /**
         * The hardware unscaler for the control space.
         */
        Handle<AdcUnscaler> adc_unscaler_control;
        /**
         * The list of channels to exclude from AdcScaler. This can be used by
         * subclasses to prevent creation of scaled slate elements that are
         * handled internally, such as dP cals in FalconControl.
         */
        str_s adc_scaler_exclude_channels;
        /**
         * The slate receiver from string 'a'.
         */
        Handle<SlateSharerReceiver> slate_receiver_a;
        /**
         * The slate receiver from string 'b'.
         */
        Handle<SlateSharerReceiver> slate_receiver_b;
        /**
         * The slate receiver from string 'c'.
         */
        Handle<SlateSharerReceiver> slate_receiver_c;
        /**
         * The slate combiner for our pan-string control data (all peers
         * and ourselves).
         */
        Handle<SlateCombiner> slate_combiner_control;
        /**
         * Command lookup table.
         */
        CommandTable cmd_table;
        /**
         * The primary state machine. This coordinates the activation and
         * deactivation of ControlTasks and the execution of
         * EventSequences.
         */
        Handle<StateMachine> state_machine;
        /**
         * Control logic associated with the TelemetryRelay.
         */
        Handle<TelemetryRelayControl> telem_relay_control;
        /**
         * Dispatches event sequences instead of state transitions in
         * response to certain alarms.
         */
        AlarmEventSequenceHandler alarm_sequence_handler;
        /**
         * Provides access to the alert system.
         */
        Handle<AlertProvider> alert_provider;
        /**
         * Checks and dispatches any sensor alarms.
         */
        Handle<SlateAlarmTask> slate_alarms;
        /**
         * External command dispatcher for the synced command inputs.
         */
        Handle<ExternalCommandDispatcher> ground_cmd_dispatcher_synced;
        /**
         * Slate command interface for control elements.
         */
        Handle<SlateCommandInterface> slate_command_interface_control;
        /**
         * Holds a disconnect command from the command system and ensures
         * it is delivered to the state machine.
         */
        Handle<CommandSpammer> disconnect_command_spammer;
        /**
         * The set of enums to be included in telemetry metadata.
         */
        Handle<EnumRegistry> enum_registry;
        /**
         * Holds all EnumeratedBitmap's. See EnumeratedBitmap.
         */
        Handle<EnumeratedBitmapManager> enumerated_bitmaps;
        /**
         * Synchronize telemetry timestamps across the vehicle.
         */
        Handle<TimestampSynchronizer> timestamp_syncer;
        /**
         * Time spent unscaling the control outputs.
         */
        CycleTimerHandle adc_unscale_timer;
    };
} /* end namespace Drone */
#endif /* BASIC_CONTROL_H */