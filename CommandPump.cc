/**
 * @author Andy Echols
 * @date   2018-09-05
 */
#include "src/flight/sat/all/fleet_client/command/CommandPump.h"
#include "src/bullwinkle/all/core/fsw.h"
namespace Drone
{
    const std::string CommandPump::accepted_seq_num_name = "accepted_seq_num";
    const std::string CommandPump::rejected_seq_num_name = "rejected_seq_num";
    /**
     * Construct the pump glue.
     *
     * @param  _start_pump    Delegate that starts pumping a command.
     */
    CommandPump::CommandPump(start_pump_t _start_pump)
        : is_init(false), start_pump(_start_pump), slate(),
          accepted_seq_num_tok(), rejected_seq_num_tok(),
          last_accepted_seq_num_tok(), last_rejected_seq_num_tok()
    {}
    /**
     * Prepare to get feedback from the feedback reader.
     *
     * @param  feedback_builder  Slate with ExternalCommandFeedbackReader
     *                           tokens supplying command feedback.
     * @param  builder           Slate used for local state.
     *
     * @return True on success.
     */
    bool CommandPump::init(SlateBuilder &feedback_builder,
                           SlateBuilder &builder)
    {
        FswAbortIf(is_init, false);
        FswAbortIfNot(
            feedback_builder.bind(accepted_seq_num_name, accepted_seq_num_tok),
            false);
        FswAbortIfNot(
            feedback_builder.bind(rejected_seq_num_name, rejected_seq_num_tok),
            false);
        FswAbortIfNot(builder.create("command_pump.last_accepted_seq_num", 0,
                                     shard_nonsync, slate_read_only,
                                     last_accepted_seq_num_tok),
                      false);
        FswAbortIfNot(builder.create("command_pump.last_rejected_seq_num", 0,
                                     shard_nonsync, slate_read_only,
                                     last_rejected_seq_num_tok),
                      false);
        slate = feedback_builder.slate(slate_no_validation);
        is_init = true;
        return true;
    }
    /**
     * Pump if there is a feedback update.
     */
    nano_t CommandPump::dispatch(nano_t control_time)
    {
        FswAbortIfNot(is_init, nano_t_max);
        if (slate[last_accepted_seq_num_tok] != slate[accepted_seq_num_tok] ||
            slate[last_rejected_seq_num_tok] != slate[rejected_seq_num_tok])
        {
            dbnprintf(100, "command pump: accepted: %u rejected %u\n",
                      slate[accepted_seq_num_tok], slate[rejected_seq_num_tok]);
            FswIfNot(start_pump(slate[accepted_seq_num_tok],
                                slate[rejected_seq_num_tok]));
            slate[last_accepted_seq_num_tok] = slate[accepted_seq_num_tok];
            slate[last_rejected_seq_num_tok] = slate[rejected_seq_num_tok];
        }
        return nano_t_max;
    }
} /* end namespace Drone */