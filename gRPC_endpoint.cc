/**
 * @author Taylor Jones
 * @date   2024-06-27
 */
#include "src/bullwinkle/all/grpc/gRPC_endpoint.h"
#include "src/bullwinkle/all/Clock.h"
namespace Drone
{
    namespace
    {
        /**
         * Equality operator for an IPv6 address struct.
         */
        bool operator==(const std::pair<in6_addr, UINT16> &lhs,
                        const std::pair<in6_addr, UINT16> &rhs)
        {
            return IN6_ARE_ADDR_EQUAL(&lhs.first, &rhs.first) &&
                   lhs.second == rhs.second;
        }
    } // namespace
    /**
     * Serialize GrpcEndpoint to string.
     *
     * @return GrpcEndpoint as string.
     */
    std::string GrpcEndpoint::to_string() const
    {
        return "grpc_endpoint: " + grpc_endpoint +
               " grpc_authority: " + grpc_authority;
    }
    /**
     * Equality.
     *
     * @param other The other object to compare for equality.
     * @return true when this object and the other object are
     * considered equal.
     */
    bool GrpcEndpoint::operator==(const GrpcEndpoint &other) const
    {
        return this->grpc_endpoint == other.grpc_endpoint &&
               this->grpc_authority == other.grpc_authority;
    }
    StaticGrpcEndpoint::StaticGrpcEndpoint(const GrpcEndpoint &__endpoint)
        : _endpoint(__endpoint)
    {}
    /**
     * Create an endpoint provider.
     *
     * @param __endpoint The static endpoint that will be returned from
     * `endpoint()`.
     * @return Endpoint provider.
     */
    std::shared_ptr<StaticGrpcEndpoint>
    StaticGrpcEndpoint::create(const GrpcEndpoint &__endpoint)
    {
        return std::shared_ptr<StaticGrpcEndpoint>(
            new StaticGrpcEndpoint{__endpoint});
    }
    /**
     * Get a static endpoint.
     *
     * @param had_failure Has no effect for static endpoints.
     *
     * @return The static endpoint.
     */
    const GrpcEndpoint &StaticGrpcEndpoint::endpoint(const bool)
    {
        return _endpoint;
    }
    /**
     * Create an endpoint provider.
     *
     * @return Endpoint provider.
     */
    std::shared_ptr<DynamicGrpcEndpoint> DynamicGrpcEndpoint::create()
    {
        return std::shared_ptr<DynamicGrpcEndpoint>(new DynamicGrpcEndpoint);
    }
    /**
     * Get an endpoint using DNS information.
     *
     * NOTE: Since information is backed by DNS, and client-side
     * failover can change the endpoint, the returned value may
     * change call to call.
     *
     * @param had_failure Indicate a failure was observed with the current
     * endpoint and perform failure actions.
     *
     * @return An endpoint backed by DNS information.
     */
    const GrpcEndpoint &DynamicGrpcEndpoint::endpoint(const bool had_failure)
    {
        const nano_t now = Clock::get_monotonic_time();
        const nano_t endpoint_age = now - slate[endpoint_created_at_tok];
        const bool use_override =
            slate[use_override_tok] || slate[force_use_override_tok];
        /*
         * If DNS is not providing any targets, return whatever we have
         * locally since there are no other options.
         *
         * NOTE: On startup, before DNS is resolved for the first time,
         * this is an expected condition.
         */
        if (!use_override && slate[num_targets_tok] == 0)
        {
            ++slate[zero_targets_tok];
            return _endpoint;
        }
        // Only move the target index if a failure is indicated.
        const UINT8 target_index_increment = had_failure ? 1 : 0;
        const UINT8 next_target_index =
            use_override ? 0
                         : (slate[target_index_tok] + target_index_increment) %
                               slate[num_targets_tok];
        /*
         * If the failover cooldown period is achieved, reset the target
         * index back to the ideal target (index zero).
         *
         * NOTE: If "failing over" when the age > cooldown, index zero
         * will be used one more time (updating `created_at`). A second
         * call/failure shortly after would then move the index to use
         * the next target.
         */
        const double cooldown_s = slate[target_index_cooldown_s_tok];
        slate[target_index_tok] =
            ((endpoint_age / dbillion) > cooldown_s) ? 0 : next_target_index;
        /*
         * Don't index off the end of the array if using information from
         * DNS.
         */
        SacAbortIf(slate[target_index_tok] >= MAX_TARGETS, _endpoint);
        /*
         * Use the override parameters if desired, otherwise use information
         * from DNS.
         */
        const ServiceTargetInfo &info =
            use_override ? override_target : targets[slate[target_index_tok]];
        const UINT16 port = slate[info.port];
        in6_addr addr{};
        addr.s6_addr32[0] = slate[info.ip[0]];
        addr.s6_addr32[1] = slate[info.ip[1]];
        addr.s6_addr32[2] = slate[info.ip[2]];
        addr.s6_addr32[3] = slate[info.ip[3]];
        std::pair<in6_addr, UINT16> new_endpoint = std::make_pair(addr, port);
        const bool endpoint_unchanged = last_endpoint == new_endpoint;
        /*
         * On failure we want to go to a new target, however, if only one target
         * is available, the endpoint won't actually change.
         *
         * So we are updating the "effective" created at time to now if the
         * endpoint has changed or a failure was indicated.
         *
         * You can also think of this as time since last failure because we use
         * this to reset the target back to index zero (0) after a cooldown
         * period.
         */
        if (!endpoint_unchanged || had_failure)
        {
            slate[endpoint_created_at_tok] = now;
        }
        /*
         * If the new ip:port matches the last ip:port,
         * the last endpoint works just fine and there is no need to do a
         * dynamic allocation to create a new host:port string.
         */
        if (endpoint_unchanged)
        {
            return _endpoint;
        }
        /*
         * Otherwise, we need to create a new endpoint using the new
         * ip:port.
         */
        last_endpoint = new_endpoint;
        /*
         * Yep, this is going to lead to an allocation.
         * If you don't like that, probably shouldn't
         * be using gRPC or protobuf either. But here
         * we are.
         */
        std::ostringstream ostr;
        /*
         * Check if the IP is a v4-mapped IP address; if it is,
         * grab the IPv4 that is mapped into the IPv6 structure.
         */
        if (IN6_IS_ADDR_V4MAPPED(&last_endpoint.first))
        {
            in_addr temp_v4{last_endpoint.first.s6_addr32[3]};
            char buf[INET_ADDRSTRLEN]{};
            inet_ntop(AF_INET, &temp_v4, buf, INET_ADDRSTRLEN);
            ostr << buf << ":" << last_endpoint.second;
        }
        else
        {
            char buf[INET6_ADDRSTRLEN]{};
            inet_ntop(AF_INET6, &last_endpoint.first, buf, INET6_ADDRSTRLEN);
            ostr << "[" << buf << "]:" << last_endpoint.second;
        }
        /*
         * Create the new string using the generated ip:port.
         */
        _endpoint.grpc_endpoint = ostr.str();
        return _endpoint;
    }
    /**
     * Initialize the endpoint provider for the specified service.
     *
     * NOTE: `dns_info_builder` allows the caller to specify where
     * in slate the DNS information will be located (since it may
     * vary node to node).
     *
     * NOTE: `instance_builder` allows specifying slate for each
     * individual instantiation of the `DynamicGrpcEndpoint`.
     *
     * @param dns_info_builder The slate builder to get DNS information
     * out of slate.
     * @param instance_builder The slate builder that will place slate
     * items at the instance level for this specific `DynamicGrpcEndpoint`.
     * @param service The service name as it appears in slate in order
     * to get the DNS information for the specific service.
     * @param endpoint The endpoint containing the authority, channel args, and
     * credentials to use.
     * @param override_port The port value to use as the default when using
     * overriden values.
     *
     * @returns true when successfully initialized.
     */
    bool DynamicGrpcEndpoint::init(SlateBuilder &dns_info_builder,
                                   SlateBuilder &instance_builder,
                                   const std::string &service,
                                   const GrpcEndpoint &__endpoint,
                                   const UINT16 override_port)
    {
        /*
         * Update the endpoint details about authentication.
         *
         * NOTE: We specifically do not save the `grpc_endpoint`
         * since we want to build that variable through DNS
         * information.
         */
        _endpoint.grpc_authority = __endpoint.grpc_authority;
        _endpoint.channel_args = __endpoint.channel_args;
        _endpoint.credentials = __endpoint.credentials;
        /*
         * Slate elements for this specific dynamic endpoint.
         */
        SlateBuilder endpoint_builder = instance_builder.sub_slate("endpoint");
        SacAbortIfNot(endpoint_builder.create("zero_targets_total",
                                              shard_nonsync, slate_read_write,
                                              zero_targets_tok),
                      false);
        SacAbortIfNot(endpoint_builder.create("target_index", shard_nonsync,
                                              slate_read_write,
                                              target_index_tok),
                      false);
        SacAbortIfNot(endpoint_builder.create("endpoint_created_at",
                                              shard_nonsync, slate_read_write,
                                              endpoint_created_at_tok),
                      false);
        SacAbortIfNot(endpoint_builder.create("target_index_cooldown_s", 900.0,
                                              shard_nonsync, slate_read_only,
                                              target_index_cooldown_s_tok),
                      false);
        /*
         * Create override tokens so that the port and IP address used to
         * create endpoints can be overriden live.
         */
        {
            SlateBuilder override_builder =
                endpoint_builder.sub_slate("override");
            SacAbortIfNot(override_builder.create("use", false, shard_nonsync,
                                                  slate_read_only,
                                                  use_override_tok),
                          false);
            SacAbortIfNot(override_builder.create(
                              "force_use", false, shard_nonsync,
                              slate_read_only, force_use_override_tok),
                          false);
            SacAbortIfNot(
                override_builder.create("port", override_port, shard_nonsync,
                                        slate_read_only, override_target.port),
                false);
            /*
             * The default override IP address is the ipv6 address of
             * cplane-server. [2620:134:b000::1:0:0]
             */
            SacAbortIfNot(override_builder.create(
                              "ip.0", htonl(0x26200134), shard_nonsync,
                              slate_read_only, override_target.ip[0]),
                          false);
            SacAbortIfNot(override_builder.create(
                              "ip.1", htonl(0xb0000000), shard_nonsync,
                              slate_read_only, override_target.ip[1]),
                          false);
            SacAbortIfNot(
                override_builder.create("ip.2", htonl(0x1), shard_nonsync,
                                        slate_read_only, override_target.ip[2]),
                false);
            SacAbortIfNot(
                override_builder.create("ip.3", htonl(0x0), shard_nonsync,
                                        slate_read_only, override_target.ip[3]),
                false);
        }
        /*
         * Create read tokens to the DNS information contained in slate.
         */
        SlateBuilder service_info_builder =
            dns_info_builder.sub_slate("service_target_info");
        SlateBuilder service_builder = service_info_builder.sub_slate(service);
        SacAbortIfNot(service_builder.bind("occupied", num_targets_tok), false);
        SlateBuilder targets_builder = service_builder.sub_slate("targets");
        /*
         * Initialize all the slate tokens for each of the targets.
         */
        for (size_t t = 0; t < MAX_TARGETS; t++)
        {
            SlateBuilder target_builder =
                targets_builder.sub_slate(std::to_string(t));
            SacAbortIfNot(target_builder.bind("port", targets[t].port), false);
            /*
             * Initialize all the slate tokens for the IP address.
             */
            for (size_t dw = 0; dw < 4; dw++)
            {
                SlateBuilder ip_builder = target_builder.sub_slate("ip");
                SacAbortIfNot(
                    ip_builder.bind(std::to_string(dw), targets[t].ip[dw]),
                    false);
            }
        }
        /*
         * Initialize slate (either of the builders is okay to use here).
         */
        slate = dns_info_builder.slate(slate_no_validation);
        return true;
    }
} // namespace Drone