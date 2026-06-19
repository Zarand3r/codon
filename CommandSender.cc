/**
 * @author Andy Echols
 * @date   2018-09-05
 */
#include "src/flight/sat/all/fleet_client/command/CommandSender.h"
#include "src/bullwinkle/all/TimestampFramedMessage.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/core/sac.h"
namespace Drone
{
    /**
     * Contstruct a command sender.
     */
    CommandSender::CommandSender()
        : is_init(false), client_node_id(unknown_node_id), relay_channels(),
          slate(), metrics_writes_tok(), errors_translate_failed_tok(),
          errors_invalid_node_tok(), errors_write_failed_tok()
    {}
    /**
     * Initialize the command sender.
     *
     * @param  builder         Slate for counters.
     * @param  client_node     This client's node name.
     * @param  _relay_channels Output channels for vehicle commands.
     *
     * @return True on success.
     */
    bool CommandSender::init(SlateBuilder &builder,
                             const std::string &client_node,
                             const node_name_channel_m &_relay_channels)
    {
        SacAbortIf(is_init, false);
        SacAbortIfNot(
            NodeIdentifier::node_name_to_node_id(client_node, client_node_id),
            false);
        SacAbortIf(_relay_channels.empty(), false);
        for (const auto &key_value : _relay_channels)
        {
            const Handle<Channel> &channel = key_value.second;
            SacAbortIfNot(channel, false);
        }
        relay_channels = _relay_channels;
        SacAbortIfNot(builder.create("metrics.writes", shard_nonsync,
                                     slate_private, metrics_writes_tok),
                      false);
        SacAbortIfNot(builder.create("errors.translate_failed", shard_nonsync,
                                     slate_read_only,
                                     errors_translate_failed_tok),
                      false);
        SacAbortIfNot(builder.create("errors.invalid_node", shard_nonsync,
                                     slate_read_only, errors_invalid_node_tok),
                      false);
        SacAbortIfNot(builder.create("errors.write_failed", shard_nonsync,
                                     slate_read_only, errors_write_failed_tok),
                      false);
        slate = builder.slate(slate_no_validation);
        is_init = true;
        return true;
    }
    /**
     * Translate a command payload to a vehicle command. Failures to translate
     * or write the command to the channel are relatively expected and tick
     * error counters and fails the command. This is so the command queue
     * doesn't get stuck retrying that failed command. (The failure to translate
     * or write itself may Sac-print.)
     *
     * @param      sequence_number  The command's sequence number.
     * @param      payload          The command's payload.
     * @param[out] issued           True if the command was successfully written
     * and false for command errors.
     *
     * @return True on success.
     */
    bool CommandSender::send(uint32_t sequence_number,
                             const CommandPayload &payload, bool &issued)
    {
        issued = false;
        SacAbortIfNot(is_init, false);
        /*
         * Convert CommandPayload representation to a raw vehicle-format
         * command message and
         */
        Message vehicle_msg;
        std::string alias;
        if (!payload.write_message(vehicle_msg, alias))
        {
            ++slate[errors_translate_failed_tok];
            return true;
        }
        SacAbortIfNot(vehicle_msg.prepare_transport(0, 0), false);
        /*
         * Wrap into a timestamp-framed vehicle format message.
         * Ask for a receipt primarily for testing on the ground.
         * Time authentication not used or supported.
         */
        TimestampFramedMessage::flags_bits_t flags;
        flags.ack = true;
        TimestampFramedMessage ts_msg;
        SacAbortIfNot(ts_msg.init(vehicle_msg, flags, 0 /* timestamp */,
                                  client_node_id, sequence_number),
                      false);
        /*
         * Send it!
         */
        Handle<Channel> channel;
        if (!map_find(relay_channels, alias, channel))
        {
            ++slate[errors_invalid_node_tok];
            return true;
        };
        if (!channel->write(ts_msg.get_transport_data(),
                            ts_msg.get_transport_data_len()))
        {
            ++slate[errors_write_failed_tok];
            return true;
        }
        ++slate[metrics_writes_tok];
        issued = true;
        return true;
    }
    /**
     * Iterate over telemetry counters.
     *
     * @param  each_counter  Called once for each telemetry counter.
     *                       Receives the counter token.
     *                       Returns true on success.
     *
     * @return True on success.
     */
    bool CommandSender::for_each_counter(
        Slot<bool, WriteToken<uint32_t>> each_counter) const
    {
        SacAbortIfNot(is_init, false);
        SacAbortIfNot(each_counter(metrics_writes_tok), false);
        SacAbortIfNot(each_counter(errors_translate_failed_tok), false);
        SacAbortIfNot(each_counter(errors_invalid_node_tok), false);
        SacAbortIfNot(each_counter(errors_write_failed_tok), false);
        return true;
    }
} /* end namespace Drone */