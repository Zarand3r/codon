/**
 * @author Pavel Chikulaev
 * @date   2018-04-17
 */
#ifndef GRPC_UTIL_H
#define GRPC_UTIL_H
#include "src/bullwinkle/all/CmdLine.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/UniqueIdGenerator.h"
#include "src/bullwinkle/all/async/DeferredCallbackQueue.h"
#include "src/bullwinkle/all/config_file.h"
#include "src/bullwinkle/all/grpc/gRPC_endpoint.h"
#include "src/bullwinkle/all/hsm/CmrtSslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/SslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/StsafeSslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/TrustZoneSslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm_type_t.enum.h"
#include <arpa/inet.h>
#include <functional>
#include <grpc++/grpc++.h>
#include <grpc++/support/async_unary_call.h>
#include <memory>
#include <netinet/in.h>
#include <sstream>
namespace Drone
{
    void init_rpc();
    bool is_rpc_initialized();
    grpc::Status aborted();
    grpc::Status create_status(grpc::StatusCode code);
    /**
     * Configuration for gRPC authentication.
     */
    struct GrpcAuthConfig
    {
        /**
         * Optional key handler hook for HSMs
         */
        key_method *key_handler = nullptr;
        /**
         * Optional HSM curve name, must be specified if
         * key handler is not nullptr.
         */
        std::string hsm_curve_name = "";
        /**
         * Path to the server CA store.
         */
        std::string ca_certs_path;
        /**
         * Path to the client cert chain.
         */
        std::string client_cert_path;
        /**
         * Path to client key (optional);
         */
        std::string client_key_path;
        /**
         * Whether to require SSL.
         */
        bool require_ssl = true;
        /**
         * Whether use client authenticaion with SSL.
         */
        bool use_client_auth = true;
        /**
         * Whether to allow use of a dummy identity if we fail to find our
         * provided one.
         */
        bool allow_bogus_identity = false;
        /**
         * Whether we're currently using a dummy identity.
         */
        bool using_bogus_identity = false;
        /**
         * Whether to use HSM for auth.
         */
        bool use_hsm = false;
        /**
         * The type of HSM used.
         */
        hsm_type_t hsm_type = hsm_type_t::undefined;
        /**
         * Path to the cipher store.
         */
        std::string stsafe_cipher_path;
        /**
         * The STsafeIdentity configuration.
         */
        StsafeIdentityConfig stsafe_config;
    };
    bool create_channel_args(const GrpcAuthConfig &auth,
                             GrpcEndpoint &endpoint);
    bool create_auth_config(GrpcAuthConfig &auth, const CmdLine &cmd,
                            const bool trustzone_available,
                            const std::string &identity_provider);
    bool create_channel_creds(GrpcAuthConfig &auth, GrpcEndpoint &endpoint);
    bool init_grpc_endpoint(GrpcAuthConfig &auth, GrpcEndpoint &endpoint);
    bool build_static_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service_name,
        std::shared_ptr<StaticGrpcEndpoint> &endpoint, GrpcAuthConfig &auth);
    bool build_static_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service_name,
        const std::string &ip_address_override,
        std::shared_ptr<StaticGrpcEndpoint> &endpoint, GrpcAuthConfig &auth);
    bool build_dynamic_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service_name,
        const std::string &service_dns_info_subslate,
        std::shared_ptr<DynamicGrpcEndpoint> &endpoint, GrpcAuthConfig &auth,
        SlateBuilder &dns_info_builder, SlateBuilder &instance_builder);
    bool init_grpc_server(
        const std::string &listen_host_port,
        const std::vector<Handle<grpc::Service>> grpc_services,
        size_t num_pollers, Handle<grpc::Server> &server,
        const std::shared_ptr<grpc::ServerCredentials> server_credentials =
            grpc::InsecureServerCredentials(),
        void *custom_key_handler = nullptr,
        const std::string &hsm_curve_name = "",
        bool ignore_validity_start_verification = false);
    Handle<grpc::ClientContext> create_context(const nano_t &timeout);
    /**
     * Interface for stub creation.
     *
     * @tparam Service gRPC service class.
     */
    template <typename Service>
    class GrpcStubFactoryInterface
    {
    public:
        /**
         * Destructor.
         */
        virtual ~GrpcStubFactoryInterface() = default;
        /**
         * Create new stub and associated channel.
         *
         * @param endpoint_provider Provides endpoints and authentication
         * parameters for new stubs.
         * @param[out] stub The stub to potentially recreate.
         * @param force Force the stub to be re-created.
         *
         * NOTE: It can be useful to force stub recreation in some situations
         * where the underlying TCP socket needs to be closed/recreated too.
         *
         * @return True if the stub was recreated.
         */
        virtual bool
        NewStub(GrpcEndpointProviderInterface &endpoint_provider,
                std::shared_ptr<typename Service::StubInterface> &stub,
                const bool force) = 0;
    };
    /**
     * Factory for creating new gRPC stubs and associated channels.
     *
     * @tparam Service gRPC service class.
     */
    template <typename Service>
    class GrpcStubFactory : public GrpcStubFactoryInterface<Service>
    {
    public:
        /**
         * Create a factory.
         *
         * @return Stub factory.
         */
        static std::shared_ptr<GrpcStubFactory> create()
        {
            return std::shared_ptr<GrpcStubFactory>(new GrpcStubFactory);
        }
        /**
         * Create new stub and associated channel.
         *
         * @param endpoint_provider Provides endpoints and authentication
         * parameters for new stubs.
         * @param[out] stub The stub to potentially recreate.
         * @param force Force the stub to be re-created.
         *
         * @return True if the stub was recreated.
         */
        bool NewStub(GrpcEndpointProviderInterface &endpoint_provider,
                     std::shared_ptr<typename Service::StubInterface> &stub,
                     const bool force) override
        {
            const GrpcEndpoint &endpoint = endpoint_provider.endpoint(force);
            /*
             * Re-create the stub if forced or if the the endpoint the stub
             * should connect to has changed.
             */
            if (force || endpoint.grpc_endpoint != last_grpc_endpoint)
            {
                /*
                 * Update the last used endpoint only when it changes.
                 */
                if (endpoint.grpc_endpoint != last_grpc_endpoint)
                {
                    last_grpc_endpoint = endpoint.grpc_endpoint;
                }
                stub = Service::NewStub(grpc::CreateCustomChannel(
                    endpoint.grpc_endpoint, endpoint.credentials,
                    endpoint.channel_args));
                return true;
            }
            /*
             * The stub did not need to be recreated.
             */
            return false;
        }

    private:
        /**
         * Constructor.
         */
        GrpcStubFactory() = default;
        /// The last stub created.
        std::string last_grpc_endpoint;
    };
    /**
     * Redirects execution of a handler function implementing a gRPC
     * service method to the thread that owns the given callback queue.
     *
     * Note that gRPC method is executed on gRPC thread and must be dispatched
     * to the main thread. This function does exactly that: schedules call on
     * the main thread and waits until it is done.
     *
     * @tparam TController  The controller type.
     * @tparam TRequest     The request type.
     * @tparam TResponse    The response type.
     * @param controller    The controller.
     * @param handler       The Simple RPC handler - member-function pointer.
     * @param queue         The deferred callback handler.
     * @param request       The request.
     * @param[out] response The response.
     *
     * @note controller and handler are passed separately as make_slot is not
     * thread safe.
     *
     * @return Status of the operation.
     */
    template <typename TController, typename TRequest, typename TResponse>
    grpc::Status dispatch_grpc_to_main_thread(
        TController &controller,
        bool (TController::*handler)(const TRequest &, grpc::StatusCode &,
                                     TResponse &),
        DeferredCallbackQueue &queue, const TRequest *request,
        TResponse *response)
    {
        FswAbortIfNot(request, aborted());
        FswAbortIfNot(response, aborted());
        grpc::StatusCode code = {};
        /*
         * Using std::bind instead of make_slot/slot_bind as they are not
         * thread safe.
         */
        auto callback = std::bind(handler, &controller, std::cref(*request),
                                  std::ref(code), std::ref(*response));
        std::future<bool> future = queue.enqueue(callback);
        /*
         * Waiting indefinitely until main thread is down with it, because we
         * don't want to continue scheduling callbacks on the main thread
         * while it is still processing requests to make behavior as single
         * threaded as possible.
         */
        FswAbortIfNot(future.get(), aborted());
        return create_status(code);
    };
    /**
     * Redirects execution of a handler function implementing a gRPC
     * service method to the thread that owns the given callback queue.
     *
     * Note that gRPC method is executed on gRPC thread and must be dispatched
     * to the main thread. This function does exactly that: schedules call on
     * the main thread and waits until it is done.
     *
     * @tparam TController  The controller type.
     * @tparam Types        The function types.
     * @param controller    The controller.
     * @param handler       The Simple RPC handler - member-function pointer.
     * @param queue         The deferred callback handler.
     * @param args          The arguments.
     *
     * @note controller and handler are passed separately as make_slot is not
     * thread safe.
     *
     * @return Status of the operation.
     */
    template <typename TController, typename... Types>
    bool dispatch_to_main_thread(TController &controller,
                                 bool (TController::*handler)(Types &...),
                                 DeferredCallbackQueue &queue, Types &...args)
    {
        /*
         * Using std::bind instead of make_slot/slot_bind as they are not
         * thread safe.
         */
        auto callback = std::bind(handler, &controller, args...);
        std::future<bool> future = queue.enqueue(callback);
        /*
         * Waiting indefinitely until main thread is down with it, because we
         * don't want to continue scheduling callbacks on the main thread
         * while it is still processing requests to make behavior as single
         * threaded as possible.
         */
        FswAbortIfNot(future.get(), false);
        return true;
    };
    /**
     * DEPRECATED: Use GrpcClient instead.
     *
     * TODO(TRAC-21796): Delete.
     *
     * gRPC client async callback dispatcher.
     * Registers callbacks and runs a gRPC completion queue loop in separate
     * thread and once server sends response, schedules callback on the main
     * thread through a deferred callback queue.
     */
    class GRPCAsyncCallbackDispatcher
    {
    public:
        GRPCAsyncCallbackDispatcher(DeferredCallbackQueue &_callback_queue);
        ~GRPCAsyncCallbackDispatcher();
        bool init();
        typedef std::function<bool()> callback_t;
        bool add_callback(std::function<bool()> callback, void *tag);
        bool remove_callback(void *tag);
        grpc::CompletionQueue &get_completion_queue();

    private:
        static bool worker_thread_func(GRPCAsyncCallbackDispatcher *this_);
        /**
         * Whether instance is initialized.
         */
        bool is_init;
        /**
         * The worker thread that runs completion queue.
         */
        std::thread thread;
        /**
         * The deferred callback queue to schedule callbacks of the thread.
         */
        DeferredCallbackQueue &callback_queue;
        /**
         * The gRPC completion queue.
         */
        grpc::CompletionQueue completion_queue;
        /**
         * The mutex to protect callbacks.
         */
        std::mutex mutex;
        /**
         * The callbacks map and whether they are still active.
         *
         * Note: I can't simply use std::map<void*, callback_t> and reset
         * the stored callback, as this might cause ABA problem as there is no
         * guarantee that deletion of callback won't delete object hidden behind
         * the void* key (this is the case for current key generation). In that
         * case, there is very small chance that new object will be allocated
         * at the same address, but insert will fail with two keys before a
         * response from server is returned and callback is actually deleted.
         */
        std::map<void *, std::pair<callback_t, bool>> callbacks;
    };
    /**
     * DEPRECATED: Use GrpcClient instead.
     *
     * TODO(TRAC-21796): Delete.
     *
     * Starts and cancels async RPCs, posts response callbacks onto the given
     * callback dispatcher.
     *
     * @note: Make sure to call PrepareAsyncXXX function, not AsyncXXX.
     *
     * Example usage:
     *
     * Handle<grpc::ClientContext> context = create_context(timeout);
     *
     * void* tag = nullptr;
     *
     * FswAbortIfNot(
     *       runner.register_request<SearchPathGeneratorServiceAPI::CalculateResponse>(
     *           context,
     *           search_path_service_stub->PrepareAsyncCalculate(
     *               context.get(), request, &runner.get_completion_queue()),
     *           make_slot(*this, &PerSatelliteState::on_response),
     *           tag),
     *       false);
     */
    class GRPCAsyncRunner : public SignalHandler
    {
        /**
         * The request information. Holds information about the incomplete
         * requests.
         *
         * @tparam TResponse The response type.
         */
        template <class TResponse>
        struct RequestInfo
        {
            /**
             * The request id.
             */
            std::string request_id;
            /**
             * The response.
             */
            TResponse response;
            /**
             * The response status.
             */
            grpc::Status status;
            /**
             * Client context of the call.
             */
            Handle<grpc::ClientContext> context;
        };

    public:
        /**
         * The constructor.
         *
         */
        GRPCAsyncRunner(GRPCAsyncCallbackDispatcher &_dispatcher)
            : dispatcher(_dispatcher)
        {}
        /**
         * Templated response callback type definition.
         *
         * @tparam TResponse Protobuf response type.
         *
         * @param tag A unique tag to identify request.
         * @param status Response status.
         * @param response Response object.
         *
         * @return True on success, false otherwise.
         */
        template <typename TResponse>
        using ResponseCallback =
            std::function<bool(void * /* tag */,
                               const grpc::Status & /* status */,
                               const TResponse & /* response */)>;
        /**
         * Register async gRPC request and callback to execute upon response
         * or timeout or other error.
         *
         * @tparam TResponse      The response type.
         * @param context         The client context of the request.
         * @param rpc             The async request object.
         * @param callback        The callback.
         * @param[out] request_id The request id.
         *
         * @return True on success.
         */
        template <typename TResponse>
        bool register_request(
            Handle<grpc::ClientContext> context,
            std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<TResponse>>
                rpc,
            GRPCAsyncRunner::ResponseCallback<TResponse> callback,
            void *&request_id)
        {
            Handle<RequestInfo<TResponse>> info(new RequestInfo<TResponse>);
            info->request_id = unique_request_id_generator.generate_id();
            info->context = context;
            request_id = &info->request_id;
            auto callback_wrapper = std::bind(
                &GRPCAsyncRunner::response_callback_trampoline<TResponse>,
                callback, info);
            FswAbortIfNot(dispatcher.add_callback(callback_wrapper, request_id),
                          false);
            rpc->StartCall();
            rpc->Finish(&info->response, &info->status, request_id);
            return true;
        }
        /**
         * Forgets request, doesn't really cancel it if it was already sent.
         *
         * @param request_id The request id.
         *
         * @return True on success.
         */
        bool forget_request(void *request_id)
        {
            FswAbortIfNot(dispatcher.remove_callback(request_id), false);
            return true;
        }
        /**
         * Returns completion queue object to initiate async requests.
         *
         * @return Completion queue object
         */
        grpc::CompletionQueue &get_completion_queue()
        {
            return dispatcher.get_completion_queue();
        }

    private:
        /**
         * Helper function to call the actual callback. Stores request info so
         * if there is slot to this function with bound request info it will
         * prolong its lifetime.
         *
         * @tparam TResponse The request type.
         * @param callback   The callback.
         * @param info       The request info.
         *
         * @return True on success.
         */
        template <typename TResponse>
        static bool response_callback_trampoline(
            GRPCAsyncRunner::ResponseCallback<TResponse> callback,
            Handle<RequestInfo<TResponse>> info)
        {
            FswAbortIfNot(
                callback(&info->request_id, info->status, info->response),
                false);
            return true;
        }
        /**
         * gRPC async callback dispatcher.
         */
        GRPCAsyncCallbackDispatcher &dispatcher;
        /**
         * Unique request id generator.
         */
        UniqueIdGenerator unique_request_id_generator;
    };
} // namespace Drone
#endif