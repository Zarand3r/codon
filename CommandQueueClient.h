/**
 * @author  Andy Echols
 * @date    2018-08-22
 */
#ifndef COMMAND_SERVICE_CLIENT_H
#define COMMAND_SERVICE_CLIENT_H
#include "drone/api/satellites/fleet/services/command_queue/service.grpc.pb.h"
#include "drone/api/satellites/fleet/services/command_queue/service.pb.h"
#include "drone/api/satellites/fleet/services/security/signed_data.pb.h"
#include "drone/api/satellites/fleet/services/security/tbs_command.pb.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/flight/sat/all/common/Ed25519Crypto.h"
#include "src/flight/sat/all/common/grpc/GrpcPoller.h"
#include "src/flight/sat/all/fleet_client/command/CommandPayload.h"
/**
 * Namespace containing the vehicle command service interfaces and types.
 */
namespace CommandServiceAPI =
    Drone::API::Satellites::Fleet::Services::CommandQueue;
/**
 * Stub interface type for the pump interface containing PumpQueue().
 */
using PumpStubInterface = CommandServiceAPI::CommandPumpService::StubInterface;
/**
 * Forward declaration of the friendly unit test.
 */
class CommandServiceClient_uto;
namespace Drone
{
    /**
     * Client of the vehicle command service's PumpQueue() API.
     * Trades command feedback for the next command to send, if any.
     *
     * Starts off disabled, and doesn't do any gRPCs until enabled (indicating
     * that it has received a Slate-share with the current settings).
     */
    class CommandQueueClient : public SignalHandler
    {
    public:
        using poller_t = GrpcPoller<CommandServiceAPI::CommandPumpService,
                                    CommandServiceAPI::PumpQueueRequest,
                                    CommandServiceAPI::PumpQueueResponse>;
        /**
         * GrpcPoller type.
         */
        CommandQueueClient(std::string _target_entity);
        bool init(SlateBuilder &builder, Handle<GrpcPollerInterface> _poller,
                  GroundNumericFlow *flow);
        bool pump_now(uint32_t accepted_sequence_num,
                      uint32_t rejected_sequence_num);
        nano_t dispatch(nano_t control_time);
        CommandServiceAPI::PumpQueueRequest build_request();
        bool
        on_pump_response(const grpc::Status &status,
                         const CommandServiceAPI::PumpQueueResponse &response);
        /**
         * A pump call returned with a command payload.
         *
         * @param  sequence_num  Sequence number of the command to send.
         * @param  payload       The sequence number of the command.
         * @param  issued        Whether the command was successfully issued.
         *
         * @return True on success.
         */
        Signal<bool, uint32_t, const CommandPayload &, bool &> send_command_sig;
        /**
         * Keystore of noc command signing public keys.
         */
        Ed25519KeyStore<
            Drone::API::Satellites::Fleet::Services::Security::TBSCommand>
            noc_keystore;
        /**
         * Keystore of command authorization command signing public keys.
         */
        Ed25519KeyStore<
            Drone::API::Satellites::Fleet::Services::Security::SignedData>
            command_auth_keystore;

    private:
        SX_DISALLOW_COPY_AND_ASSIGN(CommandQueueClient);
        /**
         * Whether the reader is successfully inited.
         */
        bool is_init;
        /**
         * Entity name of this vehicle in the pump service.
         */
        const std::string target_entity;
        /**
         * Client for issuing async PumpQueue RPCs.
         */
        Handle<GrpcPollerInterface> poller;
        /**
         * Slate containing the following tokens.
         */
        Slate slate;
        /**
         * The the sequence number of the last command returned. Zero if never
         * called or the last call returned no command to send.
         */
        WriteToken<uint32_t> last_send_sequence_num_tok;
        /**
         * Last failed send command sequence number.
         */
        WriteToken<uint32_t> send_failed_sequence_num_tok;
        /**
         * Number of calls that returned with OK status and a command to send.
         */
        WriteToken<uint32_t> returned_with_command_tok;
        /**
         * Last accepted command sequence number.
         */
        WriteToken<uint32_t> accepted_sequence_number_tok;
        /**
         * Last rejected command sequence number.
         */
        WriteToken<uint32_t> rejected_sequence_number_tok;
        /**
         * Last accepted command sequence number included in request to ground.
         */
        WriteToken<uint32_t> last_request_accepted_seq_num_tok;
        /**
         * Last rejected command sequence number included in request to ground.
         */
        WriteToken<uint32_t> last_request_rejected_seq_num_tok;
        /**
         * Number of times a command was sent without the requested command
         * type.
         */
        WriteToken<uint32_t> wrong_signature_state_tok;
        /**
         * Number of times the sequence number of a signed command did not match
         * the expected value.
         */
        WriteToken<uint32_t> signed_seq_num_check_failed_tok;
        /**
         * Number of times the ephemeral key id did not match the TBS command id
         */
        WriteToken<uint32_t> ephemeral_id_check_failed_tok;
        /**
         * Number of times invalid CommandPayload bytes were sent in a command.
         */
        WriteToken<uint32_t> invalid_command_payload_encoding_tok;
        /**
         * Number of times the wrong target_id was sent in a command.
         */
        WriteToken<uint32_t> target_id_check_failed_tok;
        /*
         * True if command polling is enabled.
         */
        ReadToken<bool> enabled_tok;
        /*
         * Whether commands must be signed.
         */
        ReadToken<bool> require_signed_command_tok;
        friend class ::CommandServiceClient_uto;
    };
} /* end namespace Drone */
#endif /* COMMAND_SERVICE_CLIENT_H */