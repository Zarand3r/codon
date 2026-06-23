/**
 * @author  Andy Echols
 * @date    2018-08-22
 */
#include "src/flight/sat/all/fleet_client/command/CommandQueueClient.h"
using namespace Drone::API::Satellites::Fleet::Services::Security;
namespace Drone
{
    /**
     * Construct a vehicle command service client.
     *
     * @param  _target_entity  Entity name of this vehicle in the pump service.
     */
    CommandQueueClient::CommandQueueClient(std::string _target_entity)
        : is_init(false), target_entity(std::move(_target_entity)), poller(),
          slate(), last_send_sequence_num_tok(), send_failed_sequence_num_tok(),
          returned_with_command_tok(), accepted_sequence_number_tok(),
          rejected_sequence_number_tok(), last_request_accepted_seq_num_tok(),
          last_request_rejected_seq_num_tok(), wrong_signature_state_tok(),
          signed_seq_num_check_failed_tok(), ephemeral_id_check_failed_tok(),
          invalid_command_payload_encoding_tok(), target_id_check_failed_tok(),
          enabled_tok(), require_signed_command_tok()
    {}
    /**
     * Initialize the client.
     *
     * @param  builder         (Sub)slate for client tokens.
     * @param  _poller         Client for issuing PumpQueue RPCs.
     * @param  flow            Telemetry flow to add metrics to.
     *
     */
    bool CommandQueueClient::init(SlateBuilder &builder,
                                  Handle<GrpcPollerInterface> _poller,
                                  GroundNumericFlow *flow)
    {
        FswAbortIf(is_init, false);
        poller = _poller;
        FswAbortIfNot(poller, false);
        SlateBuilder operator_keystore_slate = builder.sub_slate("operator_keystore");
        FswAbortIfNot(operator_keystore.init(operator_keystore_slate), false);
        SlateBuilder command_auth_keystore_slate =
            builder.sub_slate("command_auth_keystore");
        FswAbortIfNot(command_auth_keystore.init(command_auth_keystore_slate),
                      false);
        FswAbortIfNot(builder.create("last_send_sequence_num", shard_sync,
                                     slate_read_only,
                                     last_send_sequence_num_tok),
                      false);
        FswAbortIfNot(builder.create("send_failed_sequence_num", 0, shard_sync,
                                     slate_read_only,
                                     send_failed_sequence_num_tok),
                      false);
        FswAbortIfNot(builder.create("returned_with_payload", shard_sync,
                                     slate_read_only,
                                     returned_with_command_tok),
                      false);
        FswAbortIfNot(builder.create("last_request_accepted_seq_num", 0,
                                     shard_sync, slate_read_only,
                                     last_request_accepted_seq_num_tok),
                      false);
        FswAbortIfNot(builder.create("last_request_rejected_seq_num", 0,
                                     shard_sync, slate_read_only,
                                     last_request_rejected_seq_num_tok),
                      false);
        FswAbortIfNot(builder.create("accepted_sequence_number", 0, shard_sync,
                                     slate_read_only,
                                     accepted_sequence_number_tok),
                      false);
        FswAbortIfNot(builder.create("rejected_sequence_number_tok", 0,
                                     shard_sync, slate_read_only,
                                     rejected_sequence_number_tok),
                      false);
        FswAbortIfNot(builder.create("wrong_signature_state", 0, shard_sync,
                                     slate_read_only,
                                     wrong_signature_state_tok),
                      false);
        FswAbortIfNot(builder.create("signed_seq_num_check_failed", 0,
                                     shard_sync, slate_read_only,
                                     signed_seq_num_check_failed_tok),
                      false);
        FswAbortIfNot(builder.create("ephemeral_id_check_failed", 0, shard_sync,
                                     slate_read_only,
                                     ephemeral_id_check_failed_tok),
                      false);
        FswAbortIfNot(builder.create("invalid_command_payload_encoding", 0,
                                     shard_sync, slate_read_only,
                                     invalid_command_payload_encoding_tok),
                      false);
        FswAbortIfNot(builder.create("target_id_check_failed", 0, shard_sync,
                                     slate_read_only,
                                     target_id_check_failed_tok),
                      false);
        FswAbortIfNot(builder.bind("enabled", enabled_tok), false);
        FswAbortIfNot(
            builder.bind("require_signed_command", require_signed_command_tok),
            false);
        slate = builder.slate(slate_no_validation);
        if (flow)
        {
            FswAbortIfNot(flow->add_slate_element(last_send_sequence_num_tok),
                          false);
            FswAbortIfNot(flow->add_slate_element(returned_with_command_tok),
                          false);
        }
        is_init = true;
        return true;
    }
    /**
     * Update accepted / rejected sequence numbers, and call the PumpQueue() API
     * as soon as possible.  send_command_sig is signaled if/when the call
     * successfully returns with some command payload to be sent.
     *
     * @param  accepted_sequence_num  Sequence number of the last command
     *                                accepted by the vehicle, or zero.
     * @param  rejected_sequence_num  Sequence number of the last command
     *                                rejected by the vehicle, or zero.
     *
     * @return True on success.
     */
    bool CommandQueueClient::pump_now(uint32_t accepted_sequence_num,
                                      uint32_t rejected_sequence_num)
    {
        FswAbortIfNot(is_init, false);
        slate[accepted_sequence_number_tok] = accepted_sequence_num;
        slate[rejected_sequence_number_tok] = rejected_sequence_num;
        /* Cancel previous gRPC request and try a new one. We know the old,
         * previously requested sequence number has already made it to the
         * vehicle and isn't needed anymore.
         */
        poller->force_retry();
        return true;
    }
    /**
     *  Dispatch gRPC poller to check for new commands.
     *
     * @param control_time Current control time.
     *
     * @return Time of next requested wakeup.
     */
    nano_t CommandQueueClient::dispatch(nano_t control_time)
    {
        /*
         * If we're not yet enabled, it means we haven't yet heard from the
         * control process, and thus our Slate-shared settings aren't yet
         * correct (e.g require_signed_command). Don't do anything until we hear
         * from the control process.
         */
        if (!slate[enabled_tok])
        {
            return nano_t_max;
        }
        return poller->dispatch(control_time);
    }
    /**
     * Build a request for the next command we want to pull from the ground.
     *
     * Also keep track of the last sequence numbers that were used in a request
     * in order to use them for anti-replay in on_pump_response.
     *
     * @return PumpQueueRequest
     */
    CommandServiceAPI::PumpQueueRequest CommandQueueClient::build_request()
    {
        CommandServiceAPI::PumpQueueRequest request;
        request.set_target_satellite_id(target_entity);
        request.set_accepted_sequence_num(slate[accepted_sequence_number_tok]);
        request.set_rejected_sequence_num(
            std::max(slate[rejected_sequence_number_tok],
                     slate[send_failed_sequence_num_tok]));
        request.set_require_signed(slate[require_signed_command_tok]);
        slate[last_request_accepted_seq_num_tok] =
            slate[accepted_sequence_number_tok];
        slate[last_request_rejected_seq_num_tok] =
            slate[rejected_sequence_number_tok];
        return request;
    }
    /**
     * Handle gRPC response.
     *
     * @param status gRPC status
     * @param response gRPC response
     *
     * @return True on success.
     */
    bool CommandQueueClient::on_pump_response(
        const grpc::Status &status,
        const CommandServiceAPI::PumpQueueResponse &response)
    {
        FswAbortIfNot(is_init, false);
        if (!status.ok())
        {
            return true;
        }
        const uint32_t seq_num = response.send_sequence_number();
        slate[last_send_sequence_num_tok] = seq_num;
        if (response.has_payload() || response.has_signed_command())
        {
            ++slate[returned_with_command_tok];
            FswAbortIf(seq_num == 0, false);
            CommandPayload payload;
            if (slate[require_signed_command_tok])
            {
                if (!response.has_signed_command())
                {
                    ++slate[wrong_signature_state_tok];
                    return false;
                }
                /*
                 * Unwrap outer command authorization layer.
                 */
                SignedData single_signed;
                std::string outer_ephemeral_id;
                if (!command_auth_keystore.unwrap(single_signed,
                                                  outer_ephemeral_id,
                                                  response.signed_command()))
                {
                    return false;
                }
                /*
                 * Unwrap inner NOC signature layer.
                 */
                TBSCommand tbs;
                std::string inner_ephemeral_id;
                if (!operator_keystore.unwrap(tbs, inner_ephemeral_id,
                                         single_signed))
                {
                    return false;
                }
                /*
                 * Anti-replay.
                 */
                uint32_t command_id = 0;
                string_to_uint(tbs.command_id(), command_id);
                /*
                 * Reject command if it appears to have been replayed.
                 *
                 * skip this check if we have no accepted commands yet so we can
                 * accept any sequence number upon reboot.
                 *
                 */
                if (seq_num != command_id ||
                    (slate[accepted_sequence_number_tok] > 0 &&
                     command_id <=
                         std::max(slate[last_request_accepted_seq_num_tok],
                                  slate[last_request_rejected_seq_num_tok])))
                {
                    ++slate[signed_seq_num_check_failed_tok];
                    return false;
                }
                if ((!outer_ephemeral_id.empty() &&
                     (outer_ephemeral_id != tbs.creation_info_id())) ||
                    (!inner_ephemeral_id.empty() &&
                     (inner_ephemeral_id != tbs.creation_info_id())))
                {
                    ++slate[ephemeral_id_check_failed_tok];
                    return false;
                }
                if (target_entity != tbs.target_id())
                {
                    ++slate[target_id_check_failed_tok];
                    return false;
                }
                if (!CommandPayload::from_protobuf(tbs.command_payload(),
                                                   payload))
                {
                    ++slate[invalid_command_payload_encoding_tok];
                    return false;
                }
            }
            else
            {
                if (!response.has_payload())
                {
                    ++slate[wrong_signature_state_tok];
                    return false;
                }
                if (!CommandPayload::from_protobuf(response.payload(), payload))
                {
                    ++slate[invalid_command_payload_encoding_tok];
                    return false;
                }
            }
            bool issued = false;
            FswAbortIfNot(send_command_sig.emit(seq_num, payload, issued),
                          false);
            if (!issued)
            {
                slate[send_failed_sequence_num_tok] = seq_num;
            }
        }
        return true;
    }
} /* end namespace Drone */
