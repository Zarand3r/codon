/**
 * @author Pavel Chikulaev
 * @date   2017-02-17
 */
#ifndef DEFERRED_CALLBACK_QUEUE_H
#define DEFERRED_CALLBACK_QUEUE_H
#include "src/bullwinkle/all/EventLoop.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/core/AutoFd.h"
#include "src/bullwinkle/all/core/util.h"
#include <functional>
#include <future>
#include <mutex>
#include <unistd.h>
#include <vector>
namespace Drone
{
    /**
     * Thread-safe callback queue.
     *
     * This class allows execution of arbitrary callbacks on an EventLoop. This
     * is useful for synchronizing interrupts like gRPC handlers with the rest
     * of the single-threaded rocket world.
     *
     * Usage
     * =====
     *
     *   1. Create a DeferredCallbackQueue and register it with the EventLoop.
     *
     *   2. Call enqueue(...) from another thread to enqueue a function to be
     *      called on the next run of the EventLoop.
     *
     *   3. Optionally call get() on the returned future to block until the
     *      function has been executed by the main thread.
     *
     * Example
     * =======
     *
     *   // Create and register the callback queue with the EventLoop.
     *   Handle<DeferredCallbackQueue> callback_queue =
     *       DeferredCallbackQueue::create(eloop.fds);
     *
     *   // Run some code on the main thread and block until it completes.
     *   return callback_queue.enqueue([context, request, response](){
     *       // Do stuff...
     *   }).get();
     *
     * Notes
     * =====
     *
     *   * Callbacks are executed in strict FIFO order.
     *
     *   * Each dispatch() executes all callbacks which were enqueued when
     *     dispatch() began. Any callbacks enqueued after dispatch() begins
     *     have to wait until the next dispatch() call.
     *
     *   * All public interfaces are thread-safe.
     *
     */
    class DeferredCallbackQueue : public SignalHandler
    {
    public:
        static Handle<DeferredCallbackQueue> create(FdBag &fd_bag);
        ~DeferredCallbackQueue();
        /**
         * Enqueue a function to be called on next dispatch.
         *
         * @tparam Function Type of the function to call.
         * @tparam Args     Variadic argument types to forward to the function.
         * @tparam Result   Return type of the function (inferred).
         *
         * @param f    Function to call.
         * @param args Arguments to pass to the function.
         *
         * @return A std::future that will hold the function's return value.
         */
        template <typename Function, typename... Args,
                  typename Result = std::result_of_t<
                      std::decay_t<Function>(std::decay_t<Args>...)>>
        std::future<Result> enqueue(Function &&f, Args &&...args)
        {
            /*
             * Slots manipulate shared datastructures without locking.  This
             * means constructing a Slot from any thread thread other than the
             * main thread is a data race.
             */
            static_assert(
                !std::is_base_of<Slot<Result, Args...>, Function>::value,
                "Slot may only be used on the main thread.");
            auto callback = std::bind(std::forward<Function>(f),
                                      std::forward<Args>(args)...);
            auto promise = std::make_shared<std::promise<Result>>();
            std::future<Result> future = promise->get_future();
            std::function<void()> wrapped_callback =
                fulfillment_wrapper<Result>(callback, promise);
            /*
             * Enqueue the callback, holding the lock for as short as possible.
             */
            {
                std::lock_guard<std::mutex> lock(mutex);
                queue.push_back(std::move(wrapped_callback));
            }
            /*
             * Signal the eventfd to wake the EventLoop.
             *
             * This shouldn't ever fail, but if it does there's nothing to do
             * but complain loudly.
             */
            static const uint64_t one = 1;
            SacOnErrno(write(ready_fd, &one, sizeof(one)));
            return future;
        }

    private:
        DeferredCallbackQueue();
        bool init(FdBag &fd_bag);
        bool on_ready(FdEventSink &, FdEvent &);
        /**
         * Take a function and a promise, and return a function that calls the
         * given function and stores its result in the promise.
         *
         * @tparam Result Return type of the function.
         *
         * @param callback Function to call.
         * @param promise  Promise to store the function's result in.
         *
         * @return A wrapper function.
         */
        template <typename Result>
        std::function<void()>
        fulfillment_wrapper(std::function<Result()> callback,
                            std::shared_ptr<std::promise<Result>> promise)
        {
            return [callback = std::move(callback), promise]() {
                promise->set_value(callback());
            };
        }
        SX_DISALLOW_COPY_AND_ASSIGN(DeferredCallbackQueue);
        /**
         * The mutex that serializes access to the callback queue.
         */
        mutable std::mutex mutex;
        /**
         * Callbacks to execute on next dispatch().
         */
        std::vector<std::function<void()>> queue;
        /**
         * eventfd for signalling the EventLoop that we have work to do.
         */
        int ready_fd;
    };
    template <>
    std::function<void()> DeferredCallbackQueue::fulfillment_wrapper<void>(
        std::function<void()> callback,
        std::shared_ptr<std::promise<void>> promise);
} // namespace Drone
#endif