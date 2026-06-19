/**
 * @author Taylor Jones
 * @date   2024-06-27
 */
#ifndef GRPC_ENDPOINT_H
#define GRPC_ENDPOINT_H
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include <arpa/inet.h>
#include <grpc++/grpc++.h>
#include <memory>
#include <netinet/in.h>
#include <sstream>
namespace Drone
{
    /**
     * gRPC endpoint.
     */
    struct GrpcEndpoint
    {
        /**
         * gRPC endpoint (host:port).
         */
        std::string grpc_endpoint;
        /**
         * http/2 authority to use.
         *
         * If using a reverse-proxy this should be the hostname
         * of the server behind the proxy that ultimately gives
         * authoritative responses. Use an empty string to leave
         * as default.
         */
        std::string grpc_authority;
        /**
         * Custom arguments to use when creating a new channel.
         */
        grpc::ChannelArguments channel_args;
        /**
         * Grpc Credentials to use when creating a new Channel.
         *
         * i.e. SSL settings, such as using certificates,
         * or an HSM.
         */
        std::shared_ptr<grpc::ChannelCredentials> credentials;
        std::string to_string() const;
        bool operator==(const GrpcEndpoint &other) const;
    };
    /**
     * Interface for getting an endpoint (usually an ip:port)
     * for use in a gRPC client.
     */
    class GrpcEndpointProviderInterface
    {
    public:
        /**
         * Destructor.
         */
        virtual ~GrpcEndpointProviderInterface() = default;
        /**
         * Get an endpoint specifying where a gRPC client should connect
         * to.
         * @param had_failure Indicate a failure was observed with the current
         * endpoint (i.e. perform failure actions).
         *
         * @return The endpoint.
         */
        virtual const GrpcEndpoint &endpoint(const bool had_failure) = 0;
    };
    /**
     * Provides a static/fixed ip and port.
     */
    class StaticGrpcEndpoint : public GrpcEndpointProviderInterface
    {
    public:
        StaticGrpcEndpoint(const GrpcEndpoint &__endpoint);
        /**
         * Create an endpoint provider.
         *
         * @return Endpoint provider
         */
        static std::shared_ptr<StaticGrpcEndpoint>
        create(const GrpcEndpoint &__endpoint);
        const GrpcEndpoint &endpoint(const bool had_failure) override;

    private:
        /**
         * The static endpoint to return.
         */
        GrpcEndpoint _endpoint;
    };
    /**
     * Provides a dynamic ip:port from SRV and A/AAAA information.
     * Additionally provides loadbalancing and client-side
     * failover abilities.
     */
    class DynamicGrpcEndpoint : public GrpcEndpointProviderInterface
    {
    public:
        /**
         * The maximum number of targets supported in slate.
         */
        static constexpr size_t MAX_TARGETS = 6;
        static std::shared_ptr<DynamicGrpcEndpoint> create();
        const GrpcEndpoint &endpoint(const bool had_failure) override;
        bool init(SlateBuilder &dns_info_builder,
                  SlateBuilder &instance_builder, const std::string &service,
                  const GrpcEndpoint &__endpoint, const UINT16 override_port);

    private:
        DynamicGrpcEndpoint() = default;
        /**
         * Metric slate.
         */
        Slate slate;
        /**
         * Information about a "service target".
         *
         * In other words, the information from SRV and A/AAAA
         * DNS records that is necessary for a client to make
         * a connection to an endpoint.
         */
        struct ServiceTargetInfo
        {
            /**
             * The IP address split into 4 u32 integers.
             *
             * IPv4 addresses are the last double-word, which
             * can be checked by looking at the second to last
             * double-word; 0x0000FFFF indicates a v4-mapped
             * address is present (since an IPv6 address is
             * being shared here).
             *
             * @see RFC for details.
             * https://datatracker.ietf.org/doc/html/rfc5156#section-2.2
             */
            ReadToken<UINT32> ip[4];
            /**
             * The port to use for the service.
             */
            ReadToken<UINT16> port;
        };
        /**
         * The number of "service-targets" available for use in the `targets`.
         */
        ReadToken<UINT8> num_targets_tok;
        /**
         * The number of times there were zero targets available from DNS.
         */
        WriteToken<UINT16> zero_targets_tok;
        /**
         * Information about all possible targets that can be used.
         */
        ServiceTargetInfo targets[MAX_TARGETS];
        /**
         * Flag to indicate if the override parameters should be used.
         */
        ReadToken<bool> use_override_tok;
        /**
         * Flag to indicate if the override parameters should be used.
         * This is operator commandable and will be OR'd with use_override_tok
         * to determine if the override will be used.
         */
        ReadToken<bool> force_use_override_tok;
        /**
         * Override target.
         */
        ServiceTargetInfo override_target;
        /**
         * The index to the target from slate to use to
         * create the next endpoint.
         */
        WriteToken<UINT8> target_index_tok;
        /**
         * The cooldown period, in seconds, after which the
         * "target index" will be reset back to zero (the
         * ideal target to use).
         */
        ReadToken<double> target_index_cooldown_s_tok;
        /**
         * The time the endpoint was created at.
         */
        WriteToken<nano_t> endpoint_created_at_tok;
        /**
         * The last ip and port this class generated in a non-dynamically
         * allocating form. We store this more primitive type of the
         * ip:port to compare against and minimize when we have to
         * dynamically allocate memory for a new string.
         */
        std::pair<in6_addr, UINT16> last_endpoint{};
        /**
         * The endpoint.
         */
        GrpcEndpoint _endpoint;
    };
} // namespace Drone
#endif
