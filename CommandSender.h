/**
 * @author Andy Echols
 * @date   2018-09-05
 */
#ifndef COMMAND_SENDER_H
#define COMMAND_SENDER_H
#include "src/bullwinkle/all/NodeIdentifier.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/command_proxy_util.h"
#include "src/bullwinkle/all/io/Channel.h"
#include "src/flight/sat/all/fleet_client/command/CommandPayload.h"
class CommandSender_uto;
namespace Drone
{
    /**
     * Transforms a command sequence number and payload to a timestamp-framed
     * vehicle-format command message and sends it on its way.
     *
     * Could do the crypto signing in here too, for now delegated to the
     * existing crypto signing in satellite_proxy.
     */
    class CommandSender : public SignalHandler
    {
    public:
        using node_name_channel_m = std::map<std::string, Handle<Channel>>;
        CommandSender();
        bool init(SlateBuilder &builder, const std::string &client_node,
                  const node_name_channel_m &_relay_channels);
        bool send(uint32_t sequence_num, const CommandPayload &payload,
                  bool &issued);
        bool
        for_each_counter(Slot<bool, WriteToken<uint32_t>> each_counter) const;

    private:
        /**
         * True if init succeeded.
         */
        bool is_init;
        /**
         * Client node id to set on sent commands.
         */
        node_id_t client_node_id;
        /**
         * Output channels for vehicle commands.
         */
        node_name_channel_m relay_channels;
        /**
         * Slate for metrics.
         */
        Slate slate;
        /**
         * Number of commands successfully transformed and written to the
         * output channel.
         */
        WriteToken<uint32_t> metrics_writes_tok;
        /**
         * Number of commands that failed in translating a CommandPayload
         * message to a vehicle command message.
         */
        WriteToken<uint32_t> errors_translate_failed_tok;
        /**
         * Number of commands targeting a node that doesn't appear in the map
         * of output channels.
         */
        WriteToken<uint32_t> errors_invalid_node_tok;
        /**
         * Number of commands that failed to write to the output channel.
         */
        WriteToken<uint32_t> errors_write_failed_tok;
        friend class ::CommandSender_uto;
    };
} /* end namespace Drone */
#endif /* COMMAND_SENDER_H */