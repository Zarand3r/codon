/**
 * @author Andy Echols
 * @date   2018-09-05
 */
#ifndef COMMAND_PUMP_H
#define COMMAND_PUMP_H
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateBuilder.h"
namespace Drone
{
    /**
     * Command pump glue. Trade feedback for the next command on a timer and
     * immediately on updated feedback.
     */
    class CommandPump : public SignalHandler
    {
    public:
        /**
         * @name Slate token names.
         * @{
         */
        static const std::string accepted_seq_num_name;
        static const std::string rejected_seq_num_name;
        /**
         * @}
         */
        /**
         * Delegate to start pumping for a command.
         * @see CommandQueueClient::pump_if_not_pumping
         */
        typedef Slot<bool, uint32_t, uint32_t> start_pump_t;
        explicit CommandPump(start_pump_t _start_pump);
        bool init(SlateBuilder &feedback_builder, SlateBuilder &builder);
        nano_t dispatch(nano_t control_time);

    private:
        /**
         * True if init succeeded.
         */
        bool is_init;
        /**
         * Delegate that starts pumping a command.
         */
        start_pump_t start_pump;
        /**
         * Slate.
         */
        Slate slate;
        /**
         * @see ExternalCommandFeedbackReader::accepted_seq_num_tok
         */
        ReadToken<uint32_t> accepted_seq_num_tok;
        /**
         * @see ExternalCommandFeedbackReader::rejected_seq_num_tok
         */
        ReadToken<uint32_t> rejected_seq_num_tok;
        /**
         * @see ExternalCommandFeedbackReader::accepted_seq_num_tok
         */
        WriteToken<uint32_t> last_accepted_seq_num_tok;
        /**
         * @see ExternalCommandFeedbackReader::rejected_seq_num_tok
         */
        WriteToken<uint32_t> last_rejected_seq_num_tok;
    };
} /* end namespace Drone */
#endif /* COMMAND_PUMP_H */