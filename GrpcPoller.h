src / flight / sat / all / common / grpc /
    GrpcPoller.h

#ifndef GRPC_POLLER_H
#define GRPC_POLLER_H
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/core/sxtime.h"
#include "src/bullwinkle/all/grpc/GrpcClient.h"
#include <random>
#include <string>
    class GrpcPollerUto;
namespace Drone
{
    class GrpcPollerInterface
    {
    public:
        virtual ~GrpcPollerInterface() = default;
        virtual nano_t dispatch(nano_t control_time) = 0;
        virtual void enable_polling() = 0;
        virtual bool polling_enabled() const = 0;
        virtual void disable_polling() = 0;
        virtual void poll_asap() = 0;
        virtual void try_cancel() = 0;
        virtual void force_retry() = 0;
        virtual bool update_grpc_polling_policy(
            const nano_t _polling_period, const nano_t _min_retry_backoff,
            const nano_t _max_retry_backoff, const bool should_poll_asap = true)
        {
            return true;
        }
    };
    struct GrpcPollerConfig
    {
        /**
         * The endpoint.
         */
        std::shared_ptr<GrpcEndpointProviderInterface> endpoint_provider;
        /**
         * Timeout for gRPC calls
         */
        nano_t grpc_timeout = billion / 2;
        /**
         * Rate at which to issue gRPC calls. If set to zero, calls will only be
         * issued on demand.
         */
        nano_t grpc_polling_period = 60 * 60 * billion;
        /**
         * Rate limit on successful calls to the endpoint. This prevents us from
         * spamming the endpoint. If set to 0, no limit is enforced.
         * [nanoseconds]
         */
        nano_t min_delay_between_successful_calls_nanos = 0;
        /**
         * Minimum time to backoff after retry.
         */
        nano_t min_retry_backoff = 1 * billion;
        /**
         * Maximum time to backoff after retry.
         */
        nano_t max_retry_backoff = 32 * billion;
        /**
         * Whether to randomize the polling period.
         *
         * This adjusts the polling period by a small amount so that clients
         * phase over time and avoids cases where many clients end up
         * synchronized.
         */
        bool randomize_polling_period = true;
        /**
         * Amount of jitter added when randomizing polling period. This is only
         * used if randomize_polling_period is set to true.
         */
        double polling_period_jitter = 0.05;
        /**
         * The time range over which to randomize the timing between reenabling
         * polling and retrying a call in seconds.
         */
        nano_t reenable_polling_delay = 0;
        /**
         * Whether to randomize the reenable_polling_delay;
         */
        bool randomize_reenable_polling_delay = false;
        /**
         * Whether to apply the reenable_polling_delay from the last success
         * time instead of control_time.
         */
        bool reenable_polling_delay_from_last_success = false;
        /**
         * Whether to randomize retry backoff time.
         */
        bool randomize_retry_backoff = true;
        /**
         * Whether to start the poller as enabled.
         */
        bool start_enabled = true;
        /**
         * Whether to drain the queue FIFO instead of LIFO.
         */
        bool queue_draining_method_is_fifo = true;
        /**
         * The gRPC authentication configuration.
         */
        GrpcAuthConfig auth = {};
    };
    template <class TService, class TRequest, class TResponse>
    class GrpcPoller : public GrpcPollerInterface, public SignalHandler
    {
    public:
        using client_t = GrpcClientInterface<TService, TRequest, TResponse>;
        using request_builder_function_t = std::function<TRequest()>;
        /**
         * Static constructor.
         *
         * @param _name Name for this object.
         * @param builder SlateBuilder in which to create elements.
         * @param _config Configuration parameters.
         * @param queue Callback queue
         * @param method gRPC async method to call.
         * @param _build_request Request builder function.
         * @param _cb Callback on successful response.
         * @param flow Optional ground format telemetry flow.
         *
         * @return Handle to new GrpcPoller on success, empty Handle on failure.
         */
        static Handle<GrpcPoller>
        create(const std::string &_name, SlateBuilder builder,
               const GrpcPollerConfig &_config,
               Handle<DeferredCallbackQueue> queue,
               typename client_t::TMethod method,
               request_builder_function_t _build_request,
               typename client_t::TCallback _cb,
               GroundNumericFlow *flow = nullptr)
        {
            Handle<GrpcPoller> empty;
            Handle<GrpcPoller> poller(new GrpcPoller(_name));
            SacAbortIfNot(poller->init(builder, _config, queue, method,
                                       _build_request, _cb, flow),
                          empty);
            return poller;
        }
        /**
         * Dispatch this object:  make an async gRPC call if due.
         *
         * @param control_time Control time.
         *
         * @return Next control time.
         */
        nano_t dispatch(nano_t control_time) override
        {
            if (!slate[polling_enabled_tok] || slate[waiting_on_response_tok])
            {
                return nano_t_max;
            }
            if (just_enabled_polling)
            {
                nano_t reenable_polling_delay = config.reenable_polling_delay;
                /*
                 * Randomly distribute the reenable_polling_delay with a
                 * multiplier in the range of 0.5 to 1.5. This reduces the
                 * chance for situations where many assets re-enable their
                 * pollers at the same time, producing ringing in the number of
                 * requests to services.
                 */
                if (config.randomize_reenable_polling_delay &&
                    reenable_polling_delay)
                {
                    std::uniform_int_distribution<nano_t> dis(
                        0, reenable_polling_delay);
                    reenable_polling_delay =
                        dis(gen) + reenable_polling_delay / 2ll;
                }
                if (config.reenable_polling_delay_from_last_success)
                {
                    slate[next_poll_time_tok] =
                        slate[last_success_time_tok] + reenable_polling_delay;
                }
                else
                {
                    slate[next_poll_time_tok] =
                        control_time + reenable_polling_delay;
                }
                just_enabled_polling = false;
            }
            /*
             * Apply rate limits to the next poll time. We must ensure that the
             * next poll time never violates our rate limit.
             */
            else if (slate[next_poll_time_tok] <
                     (slate[last_success_time_tok] +
                      config.min_delay_between_successful_calls_nanos))
            {
                slate[next_poll_time_tok] =
                    slate[last_success_time_tok] +
                    config.min_delay_between_successful_calls_nanos;
            }
            /*
             * Poll if the next poll time is due.
             */
            if (control_time >= slate[next_poll_time_tok])
            {
                TRequest request = build_request();
                slate[last_poll_time_tok] = control_time;
                slate[next_poll_time_tok] = nano_t_max;
                slate[waiting_on_response_tok] = true;
                grpc_client->async(request, on_response_cb);
            }
            /*
             * This MUST return a value greater than control time.
             * Otherwise, we will consume the entire performance budget for the
             * process and starve out lower runtime-priority processes by
             * blocking for 0 time between iterations of EventLoop, which calls
             * dispatch on all event sources at the rate of the fastest one.
             */
            SacIf(slate[next_poll_time_tok] <= control_time);
            return slate[next_poll_time_tok];
        }
        /**
         * Enable polling.
         */
        void enable_polling() override
        {
            /*
             * Enable polling, and if it was previously disabled, we want to
             * queue up a poll. We check if a polling delay has been configured.
             * If so, set the next polling time according to the random delay
             * set in init(). If not, try to poll ASAP.
             */
            if (!slate[polling_enabled_tok])
            {
                slate[retry_count_tok] = 0;
                just_enabled_polling = true;
            }
            slate[polling_enabled_tok] = true;
        }
        /**
         * Accessor to check if polling is currently enabled.
         */
        bool polling_enabled() const override
        {
            return slate[polling_enabled_tok];
        }
        /**
         * Disable polling.
         */
        void disable_polling() override { slate[polling_enabled_tok] = false; }
        /**
         * Update grpc poller values
         *
         * @param _polling_period Polling period nanoseconds.
         * @param _min_retry_backoff Minimum retry backoff nanoseconds.
         * @param _max_retry_backoff Maximum retry backoff nanoseconds.
         *
         * @return True on success.
         */
        bool
        update_grpc_polling_policy(const nano_t _polling_period,
                                   const nano_t _min_retry_backoff,
                                   const nano_t _max_retry_backoff,
                                   const bool should_poll_asap = true) override
        {
            SacAbortIf(_min_retry_backoff <= 0, false);
            SacAbortIf(_max_retry_backoff <= 0, false);
            SacAbortIf(_min_retry_backoff > _max_retry_backoff, false);
            if (_polling_period == config.grpc_polling_period &&
                _min_retry_backoff == config.min_retry_backoff &&
                _max_retry_backoff == config.max_retry_backoff)
            {
                return true;
            }
            config.grpc_polling_period = _polling_period;
            nano_t polling_period = config.grpc_polling_period;
            if (config.randomize_polling_period)
            {
                const nano_t jitter = static_cast<nano_t>(
                    polling_period * config.polling_period_jitter);
                std::uniform_int_distribution<nano_t> dis(
                    polling_period - jitter, polling_period + jitter);
                polling_period = dis(gen);
            }
            slate[polling_period_tok] = polling_period;
            config.min_retry_backoff = _min_retry_backoff;
            config.max_retry_backoff = _max_retry_backoff;
            // Optionally poll ASAP if enabled to reset periods which may no
            // longer be valid.
            if (slate[polling_enabled_tok] && should_poll_asap)
            {
                poll_asap();
            }
            return true;
        }
        /**
         * Force a poll to happen on the next dispatch, or immediately following
         * the next gRPC response.
         */
        void poll_asap() override
        {
            slate[poll_asap_count_tok] += 1;
            slate[retry_count_tok] = 0;
            slate[last_poll_time_tok] = 0;
            slate[next_poll_time_tok] = 0;
        }
        /**
         * Override grpc client. This exists mostly for unit testing.
         */
        void set_client(Handle<client_t> _grpc_client)
        {
            grpc_client = _grpc_client;
        }
        /**
         * Attempt to cancel any outstanding GRPC calls by this poller.
         * Makes no guarantees that any calls are cancelled.
         */
        void try_cancel() override { grpc_client->cancel_all_outstanding(); }
        /**
         * Use try_cancel() and poll_asap() to try to cancel any existing GRPC
         * calls and retry immediately.
         */
        void force_retry() override
        {
            try_cancel();
            poll_asap();
        }

    protected:
        /**
         * Constructor
         *
         * @param _name Name.
         */
        GrpcPoller(const std::string &_name) : name(_name) {}
        /**
         * Initialize this object.
         *
         * @param builder SlateBuilder in which to create elements.
         * @param _config Configuration parameters.
         * @param queue Callback queue
         * @param method gRPC async method to call.
         * @param _build_request Request builder function.
         * @param _cb Callback on successful response.
         * @param flow Optional ground format telemetry flow.
         *
         * @return True on success.
         */
        bool init(SlateBuilder builder, const GrpcPollerConfig &_config,
                  Handle<DeferredCallbackQueue> queue,
                  typename client_t::TMethod method,
                  request_builder_function_t _build_request,
                  typename client_t::TCallback _cb,
                  GroundNumericFlow *flow = nullptr)
        {
            SacAbortIfNot(is_rpc_initialized(), false);
            config = _config;
            SacAbortIfNot(config.endpoint_provider, false);
            SacAbortIf(config.min_retry_backoff <= 0, false);
            SacAbortIf(config.max_retry_backoff <= 0, false);
            SacAbortIf(config.grpc_polling_period < 0, false);
            SacAbortIf(config.min_delay_between_successful_calls_nanos < 0,
                       false);
            SacAbortIf(config.min_delay_between_successful_calls_nanos >
                           config.grpc_polling_period,
                       false);
            build_request = _build_request;
            cb = _cb;
            SlateBuilder subslate = builder.sub_slate(name);
            slate = builder.slate(slate_no_validation);
            SacAbortIf(config.min_retry_backoff <= 0, false);
            SacAbortIf(config.max_retry_backoff <= 0, false);
            /*
             * Set up our random number generator.
             */
            std::random_device r;
            gen = std::default_random_engine(r());
            /*
             * Create the underlying grpc client.
             */
            grpc_client = GrpcClient<TService, TRequest, TResponse>::create(
                config.endpoint_provider, *queue, method, subslate,
                config.grpc_timeout, 1 /* max outstanding requests */, flow,
                {});
            SacAbortIfNot(grpc_client, false);
            /*
             * Create slate tokens.
             */
            nano_t polling_period = config.grpc_polling_period;
            SacAbortIfNot(
                subslate.create(
                    "next_poll_time_ns",
                    polling_period == 0 || !config.start_enabled ? nano_t_max
                                                                 : 0,
                    shard_nonsync, slate_read_only, next_poll_time_tok),
                false);
            SacAbortIfNot(subslate.create("last_poll_time_ns", 0, shard_nonsync,
                                          slate_read_only, last_poll_time_tok),
                          false);
            SacAbortIfNot(subslate.create("last_success_time_ns", nano_t_min,
                                          shard_nonsync, slate_read_write,
                                          last_success_time_tok),
                          false);
            SacAbortIfNot(subslate.create("polling_enabled",
                                          config.start_enabled, shard_nonsync,
                                          slate_read_only, polling_enabled_tok),
                          false);
            SacAbortIfNot(subslate.create("waiting_on_response", false,
                                          shard_nonsync, slate_read_only,
                                          waiting_on_response_tok),
                          false);
            SacAbortIfNot(subslate.create("poll_asap_count", 0, shard_nonsync,
                                          slate_read_only, poll_asap_count_tok),
                          false);
            SacAbortIfNot(subslate.create("retry_count", 0, shard_nonsync,
                                          slate_read_only, retry_count_tok),
                          false);
            SacAbortIfNot(subslate.create("hsm_type", config.auth.hsm_type,
                                          shard_nonsync, slate_read_only,
                                          hsm_type_tok),
                          false);
            if (config.randomize_polling_period)
            {
                const nano_t jitter = static_cast<nano_t>(
                    polling_period * config.polling_period_jitter);
                std::uniform_int_distribution<nano_t> dis(
                    polling_period - jitter, polling_period + jitter);
                polling_period = dis(gen);
            }
            SacAbortIfNot(subslate.create("polling_period_ns", polling_period,
                                          shard_nonsync, slate_read_write,
                                          polling_period_tok),
                          false);
            SacAbortIfNot(subslate.create("reenable_polling_delay_ns",
                                          config.reenable_polling_delay,
                                          shard_nonsync, slate_read_write,
                                          reenable_polling_delay_tok),
                          false);
            SacAbortIfNot(subslate.create("using_bogus_identity",
                                          config.auth.using_bogus_identity,
                                          shard_nonsync, slate_read_only,
                                          using_bogus_identity_tok),
                          false);
            on_response_cb =
                std::bind(&GrpcPoller::on_response, this, std::placeholders::_1,
                          std::placeholders::_2);
            if (flow)
            {
                SacAbortIfNot(flow->add_slate_element(next_poll_time_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(last_poll_time_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(last_success_time_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(polling_enabled_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(waiting_on_response_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(poll_asap_count_tok),
                              false);
                SacAbortIfNot(flow->add_slate_element(retry_count_tok), false);
                SacAbortIfNot(flow->add_slate_element(hsm_type_tok), false);
                SacAbortIfNot(flow->add_slate_element(polling_period_tok),
                              false);
                SacAbortIfNot(
                    flow->add_slate_element(reenable_polling_delay_tok), false);
                SacAbortIfNot(flow->add_slate_element(using_bogus_identity_tok),
                              false);
            }
            return true;
        }
        /**
         * Handle a gRPC response.
         *
         * On success, pass through the response to the supplied callback, and
         * set up our next polling time.
         * On failure, calculate our back off and set our next polling time.
         *
         * @return True on success.
         */
        bool on_response(const grpc::Status &status, const TResponse &response)
        {
            /*
             * If queue is FIFO, keep behavior the same, reset now.
             */
            if (config.queue_draining_method_is_fifo)
            {
                slate[waiting_on_response_tok] = false;
            }
            nano_t time_until_call = slate[polling_period_tok];
            if (status.ok())
            {
                slate[last_success_time_tok] = slate[last_poll_time_tok];
                SacIfNot(cb(status, response));
                /*
                 * If LIFO, wait until callback invoked to reset this.
                 */
                if (!config.queue_draining_method_is_fifo)
                {
                    slate[waiting_on_response_tok] = false;
                }
                if (slate[retry_count_tok] > 0)
                {
                    /*
                     * Randomly distribute the next call time within the range
                     * of 0.5 to 1.5 polling periods. This reduces the chance
                     * for situations where many assets check in at once (like
                     * after an outage) and then are scheduled to poll at the
                     * same time from then on (+- jitter) producing ringing in
                     * the number of requests to services.
                     */
                    std::uniform_int_distribution<nano_t> dis(
                        0, slate[polling_period_tok]);
                    time_until_call =
                        dis(gen) + slate[polling_period_tok] / 2ll;
                }
                slate[retry_count_tok] = 0;
                slate[poll_asap_count_tok] = 0;
            }
            else
            {
                /*
                 * If LIFO, make sure to reset this if the gRPC call failed.
                 */
                if (!config.queue_draining_method_is_fifo)
                {
                    slate[waiting_on_response_tok] = false;
                }
                UINT64 multiplier = 1ll
                                    << std::min(slate[retry_count_tok], 32u);
                /*
                 * Constrain multiplier to a value that will not overflow
                 * nano_t.
                 */
                multiplier = std::min<UINT64>(
                    nano_t_max / config.min_retry_backoff, multiplier);
                nano_t backoff =
                    std::min<UINT64>(config.min_retry_backoff * multiplier,
                                     config.max_retry_backoff);
                if (config.randomize_retry_backoff)
                {
                    std::uniform_int_distribution<nano_t> dis(
                        config.min_retry_backoff, backoff);
                    backoff = dis(gen);
                }
                time_until_call = backoff;
                slate[retry_count_tok] += 1;
            }
            if (time_until_call != 0)
            {
                slate[next_poll_time_tok] =
                    slate[last_poll_time_tok] + time_until_call;
            }
            return true;
        }
        /**
         * Name.
         */
        const std::string name;
        /**
         * The slate.
         */
        Slate slate;
        /**
         * Next time to poll.
         */
        WriteToken<nano_t> next_poll_time_tok;
        /**
         * Last time to poll.
         */
        WriteToken<nano_t> last_poll_time_tok;
        /**
         * Time of last successful call.
         */
        WriteToken<nano_t> last_success_time_tok;
        /**
         * Next time to poll.
         */
        WriteToken<bool> polling_enabled_tok;
        /**
         * Whether we're currently waiting for the gRPC client to respond.
         */
        WriteToken<bool> waiting_on_response_tok;
        /**
         * Number of times we have called poll_asap before the next successful
         * response.
         */
        WriteToken<UINT32> poll_asap_count_tok;
        /**
         * Number of retry attempts for the current call, used for calculating
         * exponential backoff.
         */
        WriteToken<UINT32> retry_count_tok;
        /**
         * Polling period, including jitter.
         */
        WriteToken<nano_t> polling_period_tok;
        /**
         * Polling delay when polling is reenabled.
         */
        WriteToken<nano_t> reenable_polling_delay_tok;
        /**
         * HSM type [hsm_type_t enum]
         */
        WriteToken<INT32> hsm_type_tok;
        /**
         * Telemetry for if we're using a dummy identity.
         */
        ReadToken<bool> using_bogus_identity_tok;
        /**
         * Grpc client.
         */
        Handle<client_t> grpc_client;
        /**
         * Config
         */
        GrpcPollerConfig config;
        /**
         * Request builder function.
         */
        request_builder_function_t build_request;
        /**
         * Callback to owner.
         */
        typename client_t::TCallback cb;
        /**
         * Callback to our on_response function.
         */
        typename client_t::TCallback on_response_cb;
        /**
         * Random number generator.
         */
        std::default_random_engine gen;
        /**
         * Temporary flag to indicate polling was just enabled and the next poll
         * should be scheduled.
         */
        bool just_enabled_polling = false;
        friend class ::GrpcPollerUto;
    };
} /* namespace Drone */
#endif /* GRPC_POLLER_H */