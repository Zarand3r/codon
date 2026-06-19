/**
 * @author Pavel Chikulaev
 * @date   2017-02-17
 */
#include "src/bullwinkle/all/async/DeferredCallbackQueue.h"
#include <sys/eventfd.h>
namespace Drone
{
    /**
     * Static constructor.
     *
     * @param fd_bag File descriptor bag to add our eventfd to.
     *
     * @return An initialized handle on success, or a null handle on failure.
     */
    Handle<DeferredCallbackQueue> DeferredCallbackQueue::create(FdBag &fd_bag)
    {
        Handle<DeferredCallbackQueue> callback_queue(
            new DeferredCallbackQueue());
        SacAbortIfNot(callback_queue, Handle<DeferredCallbackQueue>());
        SacAbortIfNot(callback_queue->init(fd_bag),
                      Handle<DeferredCallbackQueue>());
        return callback_queue;
    }
    /**
     * Constructor.
     */
    DeferredCallbackQueue::DeferredCallbackQueue() : SignalHandler(), queue() {}
    /**
     * Initialize the callback queue.
     *
     * @param fd_bag File descriptor bag to add our eventfd to.
     *
     * @return True on success.
     */
    bool DeferredCallbackQueue::init(FdBag &fd_bag)
    {
        /*
         * Initialize the eventfd. Set EFD_NONBLOCK so we don't block the main
         * thread if on_ready() is called without any callbacks enqueued.
         */
        SacAbortOnErrno(ready_fd = eventfd(0, EFD_NONBLOCK), false);
        Handle<FdEventSink> fes = fd_bag.fd(AutoFd(ready_fd));
        SacAbortIfNot(
            fes->add_events(fd_read_ev,
                            make_slot(*this, &DeferredCallbackQueue::on_ready)),
            false);
        return true;
    }
    /**
     * Destructor.
     */
    DeferredCallbackQueue::~DeferredCallbackQueue() {}
    /**
     * Run all callbacks which were already enqueued when the function begins.
     *
     * NOTE: Any callbacks enqueued while on_ready() is executing, including
     *       those enqueued by callbacks themselves, will be run on the next
     *       EventLoop select().
     *
     * @return True on success.
     */
    bool DeferredCallbackQueue::on_ready(FdEventSink &, FdEvent &)
    {
        /*
         * Get callbacks to run.
         */
        std::vector<std::function<void()>> callbacks;
        {
            std::lock_guard<std::mutex> lock(mutex);
            callbacks = std::move(queue);
        }
        /*
         * Run callbacks.
         */
        for (std::function<void()> &callback : callbacks)
        {
            callback();
        }
        /*
         * Clear the eventfd if no new callbacks were added.
         */
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (queue.empty())
            {
                /*
                 * Ignore failures since the read is expected to fail if the
                 * eventfd was never set in the first place.
                 */
                uint64_t unused = 0;
                read(ready_fd, &unused, sizeof(unused));
            }
        }
        return true;
    }
    /**
     * Explicit override for Result = void.
     *
     * This is necessary because
     *     promise->set_value(callback())
     * doesn't work with void callbacks.
     *
     * @see fulfillment_wrapper<Result>() for full documentation.
     */
    template <>
    std::function<void()> DeferredCallbackQueue::fulfillment_wrapper<void>(
        std::function<void()> callback,
        std::shared_ptr<std::promise<void>> promise)
    {
        return [callback = std::move(callback), promise]() {
            callback();
            promise->set_value();
        };
    }
} // namespace Drone