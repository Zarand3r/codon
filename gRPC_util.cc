/**
 * @author Pavel Chikulaev
 * @date   2018-04-17
 */
#include "src/bullwinkle/all/ServiceDirectory.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/grpc/gRPC_util.h"
void gpr_global_config_set_grpc_poll_strategy(const char *value);
void gpr_global_config_set_grpc_dns_resolver(const char *value);
namespace Drone
{
    /**
     * Whether or not we've set the pre-initialization gRPC settings.
     */
    static bool rpc_initialized = false;
    /**
     * Initializes global settings for gRPC; should be called before
     * using any other gRPC components.
     */
    void init_rpc()
    {
        /*
         * The version of linux that HOOTLs use doesn't support
         * epollex, so force everyone to use epoll1.
         */
        gpr_global_config_set_grpc_poll_strategy("epoll1");
        /*
         * c-ares fails to resolve the proxy on the vehicle for
         * unknown reasons, so force the native DNS resolver.
         */
        gpr_global_config_set_grpc_dns_resolver("native");
        rpc_initialized = true;
    }
    /**
     * @return True if gRPC is ready to be initialized.
     */
    bool is_rpc_initialized() { return rpc_initialized; }
    /**
     * A shortcut to get aborted status.
     *
     * @return Aborted gRPC status.
     */
    grpc::Status aborted() { return create_status(grpc::StatusCode::ABORTED); }
    /**
     * Converts gRPC status code to gRPC status.
     *
     * @param code The code.
     *
     * @return grpc Status which includes a code and message.
     */
    grpc::Status create_status(grpc::StatusCode code)
    {
        return grpc::Status(code, "");
    }
    /**
     * Create gRPC channel arguments with the provided parameters.
     *
     * @param config Authentication configuration.
     * @param[out] endpoint Endpoint containing the newly-created channel
     * credentials.
     *
     * @return True on success.
     */
    bool create_channel_args(const GrpcAuthConfig &auth, GrpcEndpoint &endpoint)
    {
        if (!endpoint.grpc_authority.empty())
        {
            endpoint.channel_args.SetString(GRPC_ARG_DEFAULT_AUTHORITY,
                                            endpoint.grpc_authority);
            endpoint.channel_args.SetSslTargetNameOverride(
                endpoint.grpc_authority);
        }
        /*
         * Set to -1 for "no limit".
         */
        endpoint.channel_args.SetMaxReceiveMessageSize(-1);
        endpoint.channel_args.SetMaxSendMessageSize(-1);
#ifdef SX_BORINGSSL_ENABLED
        if (auth.key_handler != nullptr)
        {
            endpoint.channel_args.SetPointer(GRPC_SSL_CUSTOM_KEY_ARG,
                                             auth.key_handler);
        }
        if (!auth.hsm_curve_name.empty())
        {
            endpoint.channel_args.SetString(GRPC_SSL_CUSTOM_HSM_CURVE_NAME_ARG,
                                            auth.hsm_curve_name);
        }
#endif
        return true;
    }
    /**
     * Create a gRPC authentication configuration.
     *
     *
     */
    bool create_auth_config(GrpcAuthConfig &auth, const CmdLine &cmd,
                            const bool trustzone_available,
                            const std::string &identity_provider)
    {
        auth.ca_certs_path = cmd.get_string("ca_certs");
        if (identity_provider == "stsafe")
        {
            auth.use_hsm = true;
            auth.hsm_type = hsm_type_t::stsafe;
            auth.client_cert_path = cmd.get_string("hsm_client_cert");
            auth.stsafe_cipher_path = cmd.get_string("stsafe_cipher");
        }
        else if (identity_provider == "trustzone" && trustzone_available)
        {
            auth.use_hsm = true;
            auth.hsm_type = hsm_type_t::trustzone;
            auth.client_cert_path = cmd.get_string("hsm_client_cert");
            auth.client_key_path = cmd.get_string("hsm_client_key");
        }
        else
        {
            auth.client_cert_path = cmd.get_string("client_cert");
            auth.client_key_path = cmd.get_string("client_key");
        }
        return true;
    }
    /**
     * Create gRPC channel credentials.
     *
     * WARNING: ☠️ Fellow travels, beware! Danger lurks below. ☠️
     *
     * Calling this repeatedly on the same object will leak
     * memory allocated for the `key_handler`.
     *
     * @param config gRPC authentication configuration settings.
     * @param[out] endpoint Endpoint containing the newly-created channel
     * credentials.
     *
     * @return True on success.
     */
    bool create_channel_creds(GrpcAuthConfig &auth, GrpcEndpoint &endpoint)
    {
        endpoint.credentials = grpc::InsecureChannelCredentials();
        /*
         * If not using SSL, nothing else is needed.
         */
        if (!auth.require_ssl)
        {
            return true;
        }
#ifdef SX_BORINGSSL_ENABLED
        auto opts = grpc::SslCredentialsOptions();
        /*
         * If SSL is required, read CA certs needed to validate server
         * cert.
         */
        SacAbortIfNot(read_str(auth.ca_certs_path, opts.pem_root_certs), false);
        /*
         * If client authentication should be used, read client cert and
         * key.
         */
        if (auth.use_client_auth)
        {
            if (!read_str(auth.client_cert_path, opts.pem_cert_chain))
            {
                /*
                 * If we fail to read the provided cert and are allowed to, use
                 * a dummy but be loud about it. This is preferable to crashing
                 * because some production cases assume processes won't crash
                 * even before we provision identities.
                 */
                if (auth.allow_bogus_identity)
                {
                    opts.pem_cert_chain = "IMAGINARY_IDENTITY!!!";
                    auth.using_bogus_identity = true;
                }
                else
                {
                    SacAbort(
                        "read_str(auth.client_cert_path, opts.pem_cert_chain)",
                        false);
                }
            }
            else
            {
                /*
                 * If we are using a HSM set the appropriate opts
                 * and config values.
                 */
                if (auth.use_hsm)
                {
                    /*
                     * This value is deliberately set to ignore because
                     * we will be using the SSL callback into the
                     * stsafe_sign function to sign with the
                     * in-hardware-only private key.
                     */
                    opts.pem_private_key = "ignore";
                    switch (auth.hsm_type)
                    {
                    case hsm_type_t::stsafe: {
                        std::string stsafe_cipher;
                        SacAbortIfNot(
                            read_str(auth.stsafe_cipher_path, stsafe_cipher),
                            false);
                        if (!auth.key_handler)
                        {
                            SacAbortIfNot(create_external_stsafe_method(
                                              stsafe_cipher, auth.key_handler,
                                              auth.stsafe_config),
                                          false);
                        }
                        switch (auth.stsafe_config.curve)
                        {
                        /*
                         * If multiple curve names are needed
                         * use config.hsm_curve_name = "P-256:P-521:P-224";
                         * etc.
                         */
                        case CurveId::NIST_P_256:
                            auth.hsm_curve_name =
                                EC_curve_nid2nist(NID_X9_62_prime256v1);
                            break;
                        case CurveId::NIST_P_384:
                            auth.hsm_curve_name =
                                EC_curve_nid2nist(NID_secp384r1);
                            break;
                        default:
                            break;
                        }
                    }
                    break;
                    case hsm_type_t::trustzone: {
                        std::string wrapped_key;
                        SacAbortIfNot(
                            read_str_binary(auth.client_key_path, wrapped_key),
                            false);
                        std::vector<uint8_t> trustzone_wrapped_key(
                            wrapped_key.begin(), wrapped_key.end());
                        if (!auth.key_handler)
                        {
                            SacAbortIfNot(
                                create_external_trustzone_method(
                                    auth.key_handler, trustzone_wrapped_key),
                                false);
                        }
                    }
                    break;
                    case hsm_type_t::cmrt: {
                        if (!auth.key_handler)
                        {
                            SacAbortIfNot(
                                create_external_cmrt_method(auth.key_handler),
                                false);
                        }
                    }
                    break;
                    case hsm_type_t::undefined:
                    default:
                        SacAbort("Invalid HSM.\n", false);
                        break;
                    }
                }
                else
                {
                    SacAbortIfNot(
                        read_str(auth.client_key_path, opts.pem_private_key),
                        false);
                }
            }
            endpoint.credentials = grpc::SslCredentials(opts);
#else
        SacMsgAbort(false, 100,
                    "SSL required on a platform that doesn't support SSL");
#endif
        }
        return true;
    }
    /**
     * Initialize a gRPC endpoint using the provided authentication config.
     *
     * @param config The authentication configuration.
     * @param[out] endpoint The endpoint to initialize.
     */
    bool init_grpc_endpoint(GrpcAuthConfig &auth, GrpcEndpoint &endpoint)
    {
        /*
         * Build the credentials for this endpoint.
         */
        SacMsgAbortIfNot(create_channel_creds(auth, endpoint), false, 500,
                         "Failed to create credentials for '%s'",
                         endpoint.grpc_endpoint.c_str());
        /*
         * Build the channel arguments for this endpoint.
         */
        SacAbortIfNot(create_channel_args(auth, endpoint), false);
        return true;
    }
    /**
     * Create a static gRPC endpoint that can be used by pollers/clients.
     *
     * This configures authentication based on command-line args, and
     * builds the "endpoint" with information provided through the static
     * service-directory.
     *
     * @param cmd Command-line arguments.
     * @param trustzone_available Trustzone available for use.
     * @param identity_provider Indicates which HSM can be used.
     * @param service The service to lookup in the static
     * service-directory.
     * @param[out] endpoint The static gRPC endpoint to configure.
     * @param[out] auth The authentication settings to use.
     *
     * @returns True on success, false otherwise.
     */
    bool build_static_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service,
        std::shared_ptr<StaticGrpcEndpoint> &endpoint, GrpcAuthConfig &auth)
    {
        /*
         * Create the authentication configuration.
         */
        SacAbortIfNot(create_auth_config(auth, cmd, trustzone_available,
                                         identity_provider),
                      false);
        /*
         * Lookup the host/authority/port in the service-directory.
         */
        SacAbortIf(service.empty(), false);
        Service grpc_service;
        SacAbortIfNot(service_directory().lookup(service, grpc_service), false);
        const std::string grpc_dest =
            grpc_service.dns_name + ":" + to_string(grpc_service.port);
        const std::string grpc_authority = grpc_service.host_name;
        /*
         * Initialize the endpoint now that we have all the necessary
         * information.
         */
        GrpcEndpoint grpc_endpoint{.grpc_endpoint = grpc_dest,
                                   .grpc_authority = grpc_authority};
        SacAbortIfNot(init_grpc_endpoint(auth, grpc_endpoint), false);
        endpoint = StaticGrpcEndpoint::create(grpc_endpoint);
        return true;
    }
    /**
     * Create a static gRPC endpoint that can be used by pollers/clients.
     *
     * This configures authentication based on command-line args, and
     * builds the "endpoint" with information provided through the static
     * service-directory but with the IP address replaced with a static IP.
     *
     * @param cmd Command-line arguments.
     * @param trustzone_available Trustzone available for use.
     * @param identity_provider Indicates which HSM can be used.
     * @param service The service to lookup in the static
     * service-directory.
     * @param ip_address_override The service IP address to use instead of DNS
     * lookup.
     * @param[out] endpoint The static gRPC endpoint to configure.
     * @param[out] auth The authentication settings to use.
     *
     * @returns True on success, false otherwise.
     */
    bool build_static_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service,
        const std::string &ip_address_override,
        std::shared_ptr<StaticGrpcEndpoint> &endpoint, GrpcAuthConfig &auth)
    {
        /*
         * Create the authentication configuration.
         */
        SacAbortIfNot(create_auth_config(auth, cmd, trustzone_available,
                                         identity_provider),
                      false);
        /*
         * Lookup the host/authority/port in the service-directory.
         */
        SacAbortIf(service.empty(), false);
        Service grpc_service;
        SacAbortIfNot(service_directory().lookup(service, grpc_service), false);
        const std::string grpc_dest =
            ip_address_override + ":" + to_string(grpc_service.port);
        const std::string grpc_authority = grpc_service.host_name;
        /*
         * Initialize the endpoint now that we have all the necessary
         * information.
         */
        GrpcEndpoint grpc_endpoint{.grpc_endpoint = grpc_dest,
                                   .grpc_authority = grpc_authority};
        SacAbortIfNot(init_grpc_endpoint(auth, grpc_endpoint), false);
        endpoint = StaticGrpcEndpoint::create(grpc_endpoint);
        return true;
    }
    /**
     * Create a dynamic gRPC endpoint, backed by DNS, that can be used by
     * pollers/clients.
     *
     * This configures authentication based on command-line args, and
     * builds the "endpoint" with an authority provided through the static
     * service-directory, but other information resovled dynamically with
     * the "dynamic service-directory" (i.e. DNS).
     *
     * @param cmd Command-line arguments.
     * @param trustzone_available Trustzone available for use.
     * @param identity_provider Indicates which HSM can be used.
     * @param service_name The service to lookup in the static
     * service-directory.
     * @param service_dns_info_subslate The subslate name the service DNS
     * information can be found at.
     * @param[out] endpoint The dynamic gRPC endpoint to configure.
     * @param[out] auth The authentication settings to use.
     * @param dns_info_builder The slate builder to access DNS information in
     * slate.
     * @param instance_builder The slate builder to save/access slate specific
     * to this dynamic endpoint.
     *
     * @returns True on success, false otherwise.
     */
    bool build_dynamic_grpc_endpoint(
        const CmdLine &cmd, const bool trustzone_available,
        const std::string &identity_provider, const std::string &service_name,
        const std::string &service_dns_info_subslate,
        std::shared_ptr<DynamicGrpcEndpoint> &endpoint, GrpcAuthConfig &auth,
        SlateBuilder &dns_info_builder, SlateBuilder &instance_builder)
    {
        /*
         * Create the authentication configuration.
         */
        SacAbortIfNot(create_auth_config(auth, cmd, trustzone_available,
                                         identity_provider),
                      false);
        /*
         * Lookup the authority in the service-directory.
         *
         * NOTE: The host/port is not used from the static service-directory
         * because that information is resolved through DNS.
         */
        SacAbortIf(service_name.empty(), false);
        Service grpc_service;
        SacAbortIfNot(service_directory().lookup(service_name, grpc_service),
                      false);
        const std::string grpc_authority = grpc_service.host_name;
        /*
         * Initialize the endpoint now that we have all the necessary
         * information.
         */
        GrpcEndpoint grpc_endpoint{.grpc_authority = grpc_authority};
        SacAbortIfNot(init_grpc_endpoint(auth, grpc_endpoint), false);
        /*
         * Create the dynamic endpoint and initialize it.
         *
         * NOTE: The static service-directory port will be used to
         * populate the default port "override" value.
         */
        endpoint = DynamicGrpcEndpoint::create();
        SacAbortIfNot(endpoint->init(dns_info_builder, instance_builder,
                                     service_dns_info_subslate, grpc_endpoint,
                                     grpc_service.port),
                      false);
        return true;
    }
    /**
     * Initializes and starts the gRPC server at the specified port.
     *
     * @param[in]   listen_host_port The host:port to listen on for connections.
     * @param[in]   grpc_services The services this server works with.
     * @param[in]   num_pollers The number of poller threads.
     * @param[out]  server The initialized and running gRPC server.
     * @param[in]   server_credentials Credentials that the gRPC server uses to
     * authenticate clients.
     * @param[in]   custom_key_handler Handler to sign messages with async
     * protocols.
     * @param[in]   ignore_validity_start_verification  Whether or not to allow
     * client X509 certificates with notBefore dates far in the future.
     * @return  True on success.
     */
    bool init_grpc_server(
        const std::string &listen_host_port,
        const std::vector<Handle<grpc::Service>> grpc_services,
        size_t num_pollers, Handle<grpc::Server> &server,
        const std::shared_ptr<grpc::ServerCredentials> server_credentials,
        void *custom_key_handler, const std::string &hsm_curve_name,
        bool ignore_validity_start_verification)
    {
        SacAbortIfNot(is_rpc_initialized(), false);
        grpc::ServerBuilder builder;
        builder.AddListeningPort(listen_host_port, server_credentials);
        for (const Handle<grpc::Service> &service : grpc_services)
        {
            builder.RegisterService(service.get());
        }
        /*
         * Set both min and max pollers so they all get created immediately.
         */
        builder.SetSyncServerOption(
            grpc::ServerBuilder::SyncServerOption::MIN_POLLERS, num_pollers);
        builder.SetSyncServerOption(
            grpc::ServerBuilder::SyncServerOption::MAX_POLLERS, num_pollers);
        /*
         * Passing in nullptr as hsm_curve_name creates default ephemeral key.
         */
        server.assume_ownership(
            builder
                .BuildAndStart(custom_key_handler,
                               (!hsm_curve_name.empty())
                                   ? hsm_curve_name.c_str()
                                   : nullptr,
                               ignore_validity_start_verification)
                .release());
        return true;
    }
    /**
     * Creates a client context with a deadline set
     * from the time of the call plus the given timeout.
     *
     * @note Always use this function to create context, or at least set a
     * deadline, we use fork of gRPC that requires to have deadline,
     * otherwise it will crash process.
     *
     * @param timeout The gRPC deadline timeout to set for requests.
     *
     * @return A handle containing a client context.
     */
    Handle<grpc::ClientContext> create_context(const nano_t &timeout)
    {
        Handle<grpc::ClientContext> context(new grpc::ClientContext);
        SacAbortIfNot(context, context);
        /*
         * set_deadline expects a time_point with a duration type matching that
         * of system_clock, which can vary by libc++ implementation -
         * specifically, clang-12 libc++ uses microseconds, whereas gcc libc++
         * uses nanoseconds. To ensure compatibility with both, always
         * duration_cast to the system_clock duration type explicitly.
         */
        const auto deadline =
            std::chrono::system_clock::now() +
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::nanoseconds(timeout));
        context->set_deadline(deadline);
        return context;
    }
    /**
     * The constructor.
     *
     * @param _callback_queue The callback queue.
     */
    GRPCAsyncCallbackDispatcher::GRPCAsyncCallbackDispatcher(
        DeferredCallbackQueue &_callback_queue)
        : is_init(false), callback_queue(_callback_queue)
    {}
    /**
     * The destructor.
     */
    GRPCAsyncCallbackDispatcher::~GRPCAsyncCallbackDispatcher()
    {
        if (is_init)
        {
            completion_queue.Shutdown();
            thread.join();
        }
    }
    /**
     * Initializes the instance.
     *
     * @return True on success.
     */
    bool GRPCAsyncCallbackDispatcher::init()
    {
        SacAbortIf(is_init, false);
        /*
         * Start separate thread.
         */
        thread =
            std::thread(&GRPCAsyncCallbackDispatcher::worker_thread_func, this);
        is_init = true;
        return true;
    }
    /**
     * Adds callback.
     *
     * @param callback The callback.
     * @param tag      The unique tag.
     *
     * @return True on success.
     */
    bool GRPCAsyncCallbackDispatcher::add_callback(callback_t callback,
                                                   void *tag)
    {
        SacAbortIfNot(is_init, false);
        std::lock_guard<std::mutex> lock(mutex);
        SacAbortIfNot(
            map_insert(callbacks, tag, std::make_pair(callback, true)), false);
        return true;
    }
    /**
     * Removes callback.
     *
     * @param tag The unique tag.
     *
     * @return True on success.
     */
    bool GRPCAsyncCallbackDispatcher::remove_callback(void *tag)
    {
        SacAbortIfNot(is_init, false);
        std::lock_guard<std::mutex> lock(mutex);
        auto it = callbacks.find(tag);
        SacAbortIf(it == callbacks.end(), false);
        it->second.second = false;
        return true;
    }
    /**
     * Returns completion queue object to initiate async requests.
     *
     * @return Completion queue object
     */
    grpc::CompletionQueue &GRPCAsyncCallbackDispatcher::get_completion_queue()
    {
        SacAssert(is_init);
        return completion_queue;
    }
    /**
     * Worker thread function that schedules registered callbacks on the main
     * thread when response is received.
     *
     * @param this_ Pointer to this class instance.
     *
     * @return True on success.
     */
    bool GRPCAsyncCallbackDispatcher::worker_thread_func(
        GRPCAsyncCallbackDispatcher *this_)
    {
        void *tag = nullptr;
        bool ok = false;
        while (this_->completion_queue.Next(&tag, &ok))
        {
            if (!ok)
            {
                continue;
            }
            std::lock_guard<std::mutex> lock(this_->mutex);
            auto it = this_->callbacks.find(tag);
            if (SacIf(it == this_->callbacks.end()))
            {
                continue;
            }
            if (it->second.second)
            {
                this_->callback_queue.enqueue(it->second.first);
            }
            this_->callbacks.erase(it);
        }
        return true;
    }
} // namespace Drone
