/**
 * @author Stefan Moluf
 * @date   03/02/11
 */
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/Clock.h"
#include "src/bullwinkle/all/core/math/math_util.h"
#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/sock_util.h"
#include <errno.h>
#include <mutex>
#include <poll.h>
#include <strings.h>
#include <sys/eventfd.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
/*
 * Define the syscall number for epoll_pwait2 if it's missing.
 * It should be available on all kernels > 5.11
 */
#ifndef __NR_epoll_pwait2
#if defined(__aarch64__) || defined(__x86_64__) || defined(__i386__) ||        \
    defined(__arm__)
#define __NR_epoll_pwait2 441
#else
#error Unsupported arch
#endif
#endif
/**
 * MUSL doesn't include an implementation for epoll_pwait2, so issue the
 * syscall directly. See man epoll_pwait(2)
 */
static int epoll_pwait2_wrapper(int epfd, struct epoll_event *events,
                                int maxevents, const struct timespec *timeout,
                                const sigset_t *sigmask)
{
    return syscall(__NR_epoll_pwait2, epfd, events, maxevents, timeout,
                   sigmask);
}
/**
 * On systems where epoll_pwait2 isn't supported, provide a fall back to
 * regular epoll_pwait.
 */
static int epoll_pwait_wrapper(int epfd, struct epoll_event *events,
                               int maxevents, const struct timespec *timeout,
                               const sigset_t *sigmask)
{
    /*
     * epoll_pwait takes a millisecond resolution timeout.  We round up to the
     * nearest millisecond. This guarantees we never wait _less_ than the
     * specified timeout and matches the behavior of select which says:
     *      Note that the timeout interval will be rounded up to the system
     *      clock.
     */
    int timeout_ms =
        timeout ? timeout->tv_sec * 1000 + (timeout->tv_nsec + 999999) / 1000000
                : -1;
    return epoll_pwait(epfd, events, maxevents, timeout_ms, sigmask);
}
/**
 * Pointer to our preferred epoll_wait implementation
 */
static int (*epoll_wait_wrapper)(int epfd, struct epoll_event *events,
                                 int maxevents, const struct timespec *timeout,
                                 const sigset_t *sigmask) = 0;
namespace Drone
{
    /**
     * Construct an FdBag.
     */
    FdBag::FdBag(size_t _max_fds)
        : max_fds(_max_fds), epoll_fd(epoll_create1(0)), fesv(),
          epoll_scratch(_max_fds), fes_count(0), active_count(0),
          event_count(0), read_event_count(0), write_event_count(0),
          except_event_count(0), error_fes(), dispatch_in_progress(false),
          fd_bound(-1)
    {
        /*
         * Reserve memory for the maximum number of file descriptors we
         * support. This avoids memory allocations when registering new file
         * descriptors in runtime.
         */
        fesv.reserve(max_fds);
        AutoFd error_fd;
        error_fes.assume_ownership(new FdEventSink(std::move(error_fd)));
        /*
         * Decide which epoll_wait implementation to use. We prefer
         * epoll_pwait2 for nanosecond resolution timeouts but it isn't
         * available on kernels older than 5.11.
         */
#ifdef __i386__
        const bool x86_32 = true;
#else
        const bool x86_32 = false;
#endif
        static std::once_flag epoll_detected;
        std::call_once(epoll_detected, [] {
            int res = epoll_pwait2_wrapper(-1, 0, 0, 0, 0);
            if (x86_32 || (res == -1 && errno == ENOSYS))
            {
                dbnprintf(100, "epoll_pwait2 is not available, falling back"
                               " to epoll_pwait\n");
                epoll_wait_wrapper = epoll_pwait_wrapper;
            }
            else
            {
                epoll_wait_wrapper = epoll_pwait2_wrapper;
            }
        });
    }
    /**
     * FdBag destructor.
     */
    FdBag::~FdBag()
    {
        /*
         * Close all connected file descriptors.
         * Calling close mutates fesv, so just repeatedly close the front until
         * it is empty.
         */
        while (!fesv.empty())
        {
            fesv.front()->close();
        }
    }
    /**
     * Returns an FdEventSink for the specified file descriptor. That
     * FdEventSink will take ownership of the file descriptor.
     *
     * Once the FdBag is managing the file descriptor, the client must
     * not call close() on the fd directly.
     *
     * @param _fd The file descriptor. The FdEventSink we create will take
     *            ownership of _fd.
     *
     * @return The Handle to the FdEventSink or #error_fes if there is
     *         an error (ie. if \a _fd is out of bounds).
     */
    Handle<FdEventSink> FdBag::fd(AutoFd _fd)
    {
        Handle<FdEventSink> fes;
        {
            /*
             * Allow FdEventSink() to be allocated at runtime. TRAC-15319 tracks
             * removing this allocation.
             */
            FswAbortIfNot(fes.assume_ownership(new FdEventSink(std::move(_fd))),
                          error_fes);
        }
        FswAbortIfNot(attach_fes(fes), error_fes);
        return fes;
    }
    /**
     * Access an FdEventSink from the bag using a raw file descriptor.
     *
     * No ownership changes occur in this function.
     *
     * @note Warning: This function has O(N) runtime over the number of fds.
     *
     * @param _fd The raw file descriptor to do the lookup with.
     *
     * @return The Handle to the FdEventSink. The Handle can be empty, if
     *         there's an error, or the FdEventSink isn't found.
     */
    Handle<FdEventSink> FdBag::get_fd(const int _fd)
    {
        if (_fd < 0)
        {
            return Handle<FdEventSink>();
        }
        for (const auto &fes : fesv)
        {
            if (fes->get_fd() == _fd)
            {
                return fes;
            }
        }
        return Handle<FdEventSink>();
    }
    /**
     * Attach an opened FdEventSink to the bag.
     *
     * @param fes FdEventSink to attach.
     *
     * @return True on success.
     */
    bool FdBag::attach_fes(Handle<FdEventSink> fes)
    {
        FswAbortIf(fes->is_closed(), false);
        FswAbortIfNot(fes->owner == NULL, false);
#if FSW_DEBUG == 1
        FswAbortIf(get_fd(fes->get_fd()), false);
#endif
        FswAbortIf(fes_count >= max_fds, false);
        const int _fd = fes->get_fd();
        fesv.push_back(fes);
        fes->owner = this;
        fes->idx = fes_count;
        ++fes_count;
        fd_bound = std::max(fd_bound, _fd);
        /*
         * When dispatch is in progress, the back of the vector may be filled
         * with items pending deletion, so swap the newly created FES into the
         * last "active" slot.
         */
        if (fes_count != fesv.size())
        {
            FswDebugAssert(dispatch_in_progress);
            std::swap(fesv.back(), fesv[fes->idx]);
        }
        fes->epoll_events.data.ptr = fes.get();
        fes->epoll_events.events = 0;
        /*
         * Some file types (regular files, /dev/null) are not pollable. Here we
         * "probe" epoll_ctl to see if it would return an EPERM error,
         * indicating a non-pollable file.
         *
         * We emulate the behavior of select (which treats these files
         * as always readable and writable) here by adding a stand-in
         * eventfd that will always poll as readable / writable.
         */
        int res =
            epoll_ctl(epoll_fd.get(), EPOLL_CTL_ADD, _fd, &fes->epoll_events);
        if (res == -1 && errno == EPERM)
        {
            int efd;
            FswAbortOnErrno((efd = eventfd(1, 0)), false);
            Handle<FdEventSink> efd_fes = fd(AutoFd(efd));
            fes->non_pollable_standin = efd_fes;
            /*
             * Update the pointer in the epoll event struct to point back at
             * the original FES. This means that epoll events generated for the
             * stand-in FD cause us to dispatch callbacks for the original FD.
             */
            efd_fes->epoll_events.data.ptr = fes.get();
            epoll_ctl(epoll_fd.get(), EPOLL_CTL_MOD, efd,
                      &efd_fes->epoll_events);
        }
        else
        {
            /*
             * update_epoll will re-add this with appropriate events, if
             * necessary.
             */
            epoll_ctl(epoll_fd.get(), EPOLL_CTL_DEL, _fd, NULL);
        }
        FswAbortIfNot(update_epoll(*fes), false);
        return true;
    }
    /**
     * Detach an FdEventSink from the bag. The caller owns the detached
     * FdEventSink after this method returns.
     *
     * @param fes FdEventSink to deattach.
     *
     * @return True on success.
     */
    bool FdBag::detach_fes(Handle<FdEventSink> fes)
    {
        FswAbortIf(fes->is_closed(), false);
        FswAbortIfNot(fes->owner == this, false);
        /*
         * Sanity checks.
         */
        FswAbortOutsideRange(fes->idx, static_cast<size_t>(0), fes_count,
                             false);
        FswAbortIfNot(fesv[fes->idx] == fes, false);
        Handle<FdEventSink> _fes;
        FswAbortIfNot(remove_fes(fes->get_fd(), fes.get(), _fes), false);
        return true;
    }
    /**
     * Select on all managed file descriptors. Will wake up at the
     * absolute time specified if no events are received.
     *
     * If wakeup is nano_t_max and there are no managed file descriptors,
     * or if there are no read and write events registered on any active
     * file descriptors, will return select_wouldblock instead of blocking
     * indefinitely. Note that selecting only on except events does not count
     * as registering a read or write event.
     *
     * @param wakeup Will wake up at this absolute time if no events
     *               are received.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_absolute(const nano_t wakeup)
    {
        nano_t time_used = nano_t_min;
        return select_absolute(wakeup, time_used);
    }
    /**
     * Select on all managed file descriptors. Will wake up at the
     * absolute time specified if no events are received.
     *
     * If wakeup is nano_t_max and there are no managed file descriptors,
     * or if there are no read and write events registered on any active
     * file descriptors, will return select_wouldblock instead of blocking
     * indefinitely. Note that selecting only on except events does not count
     * as registering a read or write event.
     *
     * @param wakeup Will wake up at this absolute time if no events
     *               are received.
     * @param[out] time_used Returns the amount of time spent
     *                       selecting on and dispatching events. Returns
     *                       0 if no events were triggered.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_absolute(const nano_t wakeup, nano_t &time_used)
    {
        return select_absolute(wakeup, fd_all_ev, time_used);
    }
    /**
     * Select on all managed file descriptors and dispatch events allowed by
     * the event mask. Will wake up at the absolute time specified if no events
     * are received.
     *
     * If wakeup is nano_t_max and there are no managed file descriptors,
     * or if there are no read and write events registered on any active
     * file descriptors, will return select_wouldblock instead of blocking
     * indefinitely. Note that selecting only on except events does not count
     * as registering a read or write event.
     *
     * @param wakeup Will wake up at this absolute time if no events
     *               are received.
     * @param fd_event_mask Mask of the event types to dispatch.
     * @param[out] time_used Returns the amount of time spent
     *                       selecting on and dispatching events. Returns
     *                       0 if no events were triggered.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_absolute(const nano_t wakeup,
                                         const uint fd_event_mask,
                                         nano_t &time_used)
    {
        /*
         * Because we filter on which events are dispatched, not which are
         * polled for, disallow a non-zero timeout.
         * This prevents unexpectedly hitting a case cases where this function
         * returns immediately (because an fd is ready) without dispatching any
         * events (because none of the ready fds pass the filter).
         */
        if (fd_event_mask != fd_all_ev)
        {
            FswAbortIfNot(wakeup == nano_t_min, select_error);
        }
        const nano_t start_time = Clock::get_monotonic_time();
        time_used = 0;
        size_t epoll_event_count = 0;
        select_code_t raw_result =
            raw_epoll(wakeup, epoll_scratch.data(), epoll_scratch.size(),
                      epoll_event_count);
        if (raw_result != select_success)
        {
            return raw_result;
        }
        /*
         * Dispatch events if successful.
         */
        FswAbortIfNot(dispatch_epoll(epoll_scratch.data(), epoll_event_count,
                                     fd_event_mask),
                      select_error);
        if (epoll_event_count > 0)
        {
            time_used = Clock::get_monotonic_time() - start_time;
        }
        return raw_result;
    }
    select_code_t FdBag::raw_epoll(nano_t wakeup,
                                   struct epoll_event *events_out,
                                   size_t max_events_out,
                                   size_t &events_out_count)
    {
        if (active_count == 0 && wakeup == nano_t_max)
        {
            return select_wouldblock;
        }
        struct timespec ts;
        struct timespec *ts_p = nullptr;
        if (wakeup != nano_t_max)
        {
            const nano_t monotonic_time = Clock::get_monotonic_time();
            const nano_t delta_timeout_ns =
                wakeup > monotonic_time ? wakeup - monotonic_time : 0;
            ts.tv_nsec = delta_timeout_ns % billion;
            ts.tv_sec = delta_timeout_ns / billion;
            ts_p = &ts;
        }
        /*
         * Use epoll_pwait2 for nanosecond resolution timeout, when available.
         */
        const int res = epoll_wait_wrapper(epoll_fd.get(), events_out,
                                           max_events_out, ts_p, NULL);
        if (res < 0)
        {
            if (errno == EINTR)
            {
                events_out_count = 0;
                return select_success;
            }
            FswErrnoAbort("epoll_wait", select_error);
        }
        events_out_count = res;
        return select_success;
    }
    bool FdBag::dispatch_epoll(struct epoll_event *events,
                               size_t epoll_event_count, uint fd_event_mask)
    {
        /*
         * Mark dispatch as "in progress". This will defer deletion of any
         * FdEventSinks until after dispatch, to avoid invalidating pointers
         * while we handle events.
         */
        dispatch_in_progress = true;
        /*
         * Clear event counts.
         */
        read_event_count = 0;
        write_event_count = 0;
        except_event_count = 0;
        /*
         * The sort here guarantees consistent dispatch order.  This primarily
         * exists for unit tests.
         */
        std::sort(events, events + epoll_event_count,
                  [&](const epoll_event &lhs, const epoll_event &rhs) {
                      FdEventSink *lp =
                          reinterpret_cast<FdEventSink *>(lhs.data.ptr);
                      FdEventSink *rp =
                          reinterpret_cast<FdEventSink *>(rhs.data.ptr);
                      return lp->get_fd() < rp->get_fd();
                  });
        const uint32_t mask = fd_event_mask & fd_epoll_ev;
        for (size_t i = 0; i < epoll_event_count; i++)
        {
            FdEventSink *fesp =
                reinterpret_cast<FdEventSink *>(events[i].data.ptr);
            FswDebugAssert(fesp->epoll_events.data.ptr == fesp);
            /*
             * Skip dispatching events if the FES has been removed.
             */
            if (fesp->owner == nullptr)
            {
                continue;
            }
            /*
             * Fill in synthentic events for HUP (other end of socket / pipe
             * closed) and errors (read end of pipe we're writing to closed).
             */
            uint32_t synth_events = 0;
            if (events[i].events & EPOLLHUP)
            {
                synth_events |= (fesp->epoll_events.events & fd_read_ev);
            }
            if (events[i].events & EPOLLERR)
            {
                synth_events |= (fesp->epoll_events.events & fd_rw_ev);
            }
            /*
             * Fill the event integer and record event counts.
             */
            const uint32_t ev = (events[i].events | synth_events) & mask;
            if (ev & fd_read_ev)
            {
                ++read_event_count;
            }
            if (ev & fd_write_ev)
            {
                ++write_event_count;
            }
            if (ev & fd_except_ev)
            {
                ++except_event_count;
            }
            /*
             * If any events were set, dispatch a new FdEvent.
             */
            if (ev)
            {
                FswAssert(fesp != nullptr);
                FswAssert(fesp->owner == this);
                FswAssert(fesp->idx < fesv.size());
                /*
                 * FdEventSink detaches itself from FdBag when closed. All valid
                 * FdEventSinks should be opened.
                 */
                FswAssert(!fesp->is_closed());
                FdEvent fev(fesp->get_fd(), ev);
                /*
                 * Store a Handle to the current FdEventSink. We will use
                 * this to detect if it changes in our dispatch call.
                 */
                if (!fesp->dispatch_event(fev))
                {
                    fesp->close();
                }
            }
        }
        /*
         * If we've deferred deletion of any FES, handle this now.
         */
        dispatch_in_progress = false;
        fesv.resize(fes_count);
        event_count +=
            read_event_count + write_event_count + except_event_count;
        return true;
    }
    /**
     * Select on all managed file descriptors. Will wake up at the
     * relative time specified if no events are received.
     *
     * If wakeup is nano_t_max and there are no managed file descriptors,
     * or if there are no read and write events registered on any active
     * file descriptors, will return select_wouldblock instead of blocking
     * indefinitely. Note that selecting only on except events does not count
     * as registering a read or write event.
     *
     * @param timeout Will wake up in this many nanoseconds if no events
     *                are received.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_relative(const nano_t timeout)
    {
        nano_t time_used = nano_t_min;
        return select_relative(timeout, time_used);
    }
    /**
     * Select on all managed file descriptors. Will wake up at the
     * relative time specified if no events are received.
     *
     * If wakeup is nano_t_max and there are no managed file descriptors,
     * or if there are no read and write events registered on any active
     * file descriptors, will return select_wouldblock instead of blocking
     * indefinitely. Note that selecting only on except events does not count
     * as registering a read or write event.
     *
     * @param timeout Will wake up in this many nanoseconds if no events
     *                are received.
     * @param[out] time_used Returns the amount of time spent
     *                       selecting on and dispatching events. Returns
     *                       0 if no events were triggered.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_relative(const nano_t timeout,
                                         nano_t &time_used)
    {
        const nano_t end_time =
            safe_nano_t_add(Clock::get_monotonic_time(), timeout);
        return select_absolute(end_time, time_used);
    }
    /**
     * Select on all managed file descriptors with an immediate timeout.
     *
     * @return Select response code. Should never return
     *         select_wouldblock.
     */
    select_code_t FdBag::select_once()
    {
        const select_code_t ret = select_absolute(nano_t_min);
        FswAssert(ret != select_wouldblock);
        return ret;
    }
    /**
     * Select on all managed file descriptors (and keep selecting) for
     * a fixed \a timeout.
     *
     * If the timeout will take us past nano_t_max, this will return
     * select_wouldblock instead of blocking indefinitely.
     *
     * @param timeout Return after this many nanoseconds.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_until_timeout(const nano_t timeout)
    {
        nano_t time_used;
        return select_until_timeout(timeout, time_used);
    }
    /**
     * Select on all managed file descriptors (and keep selecting) for
     * a fixed \a timeout.
     *
     * If the timeout will take us past nano_t_max, this will return
     * select_wouldblock instead of blocking indefinitely.
     *
     * @param timeout Return after this many nanoseconds.
     * @param[out] time_used Returns the amount of time spent
     *                       selecting on and dispatching events, less
     *                       any time spent idle at the end. Returns 0 if
     *                       no events were triggered.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_until_timeout(const nano_t timeout,
                                              nano_t &time_used)
    {
        const nano_t wake_limit = nano_t_max - timeout;
        const nano_t start_time = Clock::get_monotonic_time();
        time_used = 0;
        /*
         * When our only exit criterion is a timeout, blocking forever
         * would be bad.
         */
        if (start_time >= wake_limit)
        {
            return select_wouldblock;
        }
        const nano_t end_time = start_time + timeout;
        /*
         * Do..while in case timeout is zero. In that case, we'll at
         * least get one select() call.
         */
        do
        {
            nano_t single_time = 0;
            select_code_t ret;
            FswAbortOnSelectError(ret = select_absolute(end_time, single_time),
                                  ret);
            if (single_time > 0)
            {
                time_used = Clock::get_monotonic_time() - start_time;
            }
        } while (Clock::get_monotonic_time() < end_time);
        return select_success;
    }
    /**
     * Select on all managed file descriptors (and keep selecting) until
     * at least \a reads number of read events are dispatched. Will wake
     * up at the relative time specified if less than \a reads number of
     * read events are received.
     *
     * Will return select_error if the timeout is reached before \a
     * reads number of read events are received.
     *
     * Note that more than \a reads may be dispatched if multiple reads
     * occur in a single select() call.
     *
     * @param timeout Will wake up in this many nanoseconds if less than
     *                \a reads number of read events are received.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_until_reads(const nano_t timeout,
                                            const uint reads)
    {
        const nano_t end_time =
            safe_nano_t_add(Clock::get_monotonic_time(), timeout);
        /*
         * Keep track of the total number of reads.
         */
        uint total_reads = 0;
        while (total_reads < reads)
        {
            if (Clock::get_monotonic_time() >= end_time)
            {
                return select_error;
            }
            select_code_t ret;
            FswAbortOnSelectError(ret = select_absolute(end_time), ret);
            total_reads += read_event_count;
        }
        return select_success;
    }
    /**
     * Select on all managed file descriptors (and keep selecting) until
     * at least \a writes number of write events are dispatched. Will wake
     * up at the relative time specified if less than \a writes number of
     * write events are received.
     *
     * Will return select_error if the timeout is reached before \a
     * writes number of write events are received.
     *
     * Note that more than \a writes may be dispatched if multiple writes
     * occur in a single select() call.
     *
     * @param timeout Will wake up in this many nanoseconds if less than
     *                \a writes number of write events are received.
     *
     * @return Select response code.
     */
    select_code_t FdBag::select_until_writes(const nano_t timeout,
                                             const uint writes)
    {
        const nano_t end_time =
            safe_nano_t_add(Clock::get_monotonic_time(), timeout);
        /*
         * Keep track of the total number of writes.
         */
        uint total_writes = 0;
        while (total_writes < writes)
        {
            if (Clock::get_monotonic_time() >= end_time)
            {
                return select_error;
            }
            select_code_t ret;
            FswAbortOnSelectError(ret = select_absolute(end_time), ret);
            total_writes += write_event_count;
        }
        return select_success;
    }
    /**
     * Returns an identifier for the last file descriptor event that has
     * been dispatched. Note that this value can roll over, and is only
     * assumed to be unique for a single select() cycle.
     *
     * @return An identifier for the last file descriptor event that has
     *         been dispatched.
     */
    uint FdBag::get_event_count() const { return event_count; }
    /**
     * Returns the number of read events that have been dispatched since
     * the last select() call.
     *
     * @return The number of read events that have been dispatched since
     *         the last select() call.
     */
    uint FdBag::get_read_event_count() const { return read_event_count; }
    /**
     * Returns the number of write events that have been dispatched since
     * the last select() call.
     *
     * @return The number of write events that have been dispatched since
     *         the last select() call.
     */
    uint FdBag::get_write_event_count() const { return write_event_count; }
    /**
     * Returns the number of except events that have been dispatched since
     * the last select() call.
     *
     * @return The number of except events that have been dispatched since
     *         the last select() call.
     */
    uint FdBag::get_except_event_count() const { return except_event_count; }
    /**
     * Return one greater than the largest FD we have ever held. This is an
     * upper bound on the value of all FDs in the FdBag.
     *
     * @return Bounded FD value.
     */
    int FdBag::get_fd_bound() const { return fd_bound + 1; }
    /**
     * Select on the file descriptors from multiple FdBags
     * simultaneously. This is the non-templated version that can be used
     * if you don't know the number of FdBags at compile time.
     *
     * @param fdbags Pointer to array of pointers to FdBags we should gather
     *               file descriptors from.
     * @param fdbag_count Number of FdBag*'s in the fdbags array.
     * @param wakeup Wakeup at this absolute system time.
     * @param[out] events_dispatched Returns the number of events dispatched.
     *               Returns 0 if no events were triggered.
     *
     * @return Select response code.
     */
    select_code_t FdBag::multi_select_absolute(FdBag **fdbags,
                                               size_t fdbag_count,
                                               nano_t wakeup,
                                               size_t &events_dispatched)
    {
        struct pollfd pfd[8];
        size_t active_count = 0;
        FswAbortIfNot(fdbag_count < DIM(pfd), select_error);
        for (size_t i = 0; i < fdbag_count; ++i)
        {
            FswAbortIfNot(fdbags[i], select_error);
            const FdBag *bag = fdbags[i];
            active_count += bag->active_count;
            pfd[i].fd = bag->epoll_fd.get();
            pfd[i].events = POLLIN;
        }
        if (active_count == 0 && wakeup == nano_t_max)
        {
            return select_wouldblock;
        }
        struct timespec ts;
        struct timespec *ts_p = nullptr;
        if (wakeup != nano_t_max)
        {
            const nano_t monotonic_time = Clock::get_monotonic_time();
            const nano_t delta_timeout_ns =
                wakeup > monotonic_time ? wakeup - monotonic_time : 0;
            ts.tv_nsec = delta_timeout_ns % billion;
            ts.tv_sec = delta_timeout_ns / billion;
            ts_p = &ts;
        }
        FswAbortOnErrno(ppoll(pfd, fdbag_count, ts_p, NULL), select_error);
        select_code_t raw_result = select_success;
        for (size_t i = 0; i < fdbag_count; ++i)
        {
            if (pfd[i].revents & POLLIN)
            {
                size_t prev_count = fdbags[i]->event_count;
                select_code_t res = fdbags[i]->select_once();
                raw_result = std::max(raw_result, res);
                events_dispatched += (fdbags[i]->event_count - prev_count);
            }
        }
        return raw_result;
    }
    bool FdBag::update_epoll(FdEventSink &fes)
    {
        const int _fd = fes.get_fd();
        uint32_t old_state = fes.epoll_events.events;
        uint32_t new_state = fes.events & fd_epoll_ev;
        fes.epoll_events.events = new_state;
        if (new_state != old_state)
        {
            if (!fes.non_pollable_standin)
            {
                /*
                 * Note: there's a subtle difference between EPOLL_CTL_MOD with
                 * zero events and EPOLL_CTL_DEL, which is that even when
                 * registered for zero events, you may still receive
                 * notifications about EPOLLERR, and EPOLLHUP.
                 *
                 * We remove our interest completely (EPOLL_CTL_DEL) to avoid
                 * receiving these events when we wouldn't pay attention to
                 * them.
                 */
                int op;
                if (old_state == 0)
                {
                    op = EPOLL_CTL_ADD;
                }
                else if (new_state == 0)
                {
                    op = EPOLL_CTL_DEL;
                }
                else
                {
                    op = EPOLL_CTL_MOD;
                }
                FswAbortOnErrno(
                    epoll_ctl(epoll_fd.get(), op, _fd, &fes.epoll_events),
                    false);
            }
            else
            {
                FswAbortIfNot(
                    fes.non_pollable_standin->set_events(new_state & fd_rw_ev),
                    false);
            }
        }
        /*
         * Handle active / inactive transitions.
         * We don't consider exception events for this.
         */
        old_state &= fd_rw_ev;
        new_state &= fd_rw_ev;
        if (old_state == 0 && new_state != 0)
        {
            active_count++;
        }
        else if (new_state == 0 && old_state != 0)
        {
            active_count--;
        }
        return true;
    }
    /**
     * Remove an FdEventSink from the bag.
     *
     * @note This method returns a reference to the removed FdEventSink allowing
     * the caller to control when the FdEventSink is destroyed. It is important
     * because FdEventSink::close() can call this method while FdBag keeps
     * the only reference to the removed FdEventSink. Also note that the
     * FdEventSink may no longer hold a valid file descriptor when this
     * function is called, so the \fd argument should be used, rather than
     * getting the file descriptor from the FES.
     *
     * @param fd The file descriptor associated with the FES being removed.
     * @param fesp A pointer to the FdEventSink to remove
     * @param[out] fes Removed FdEventSink.
     *
     * @return True on success.
     */
    bool FdBag::remove_fes(int fd, FdEventSink *fesp, Handle<FdEventSink> &fes)
    {
        FswAbortIfNot(fesp, false);
        FswAbortOutsideRange(fesp->idx, static_cast<size_t>(0), fes_count,
                             false);
        FswAbortIfNot(fesv[fesp->idx], false);
        FswAbortIfNot(fesv[fesp->idx]->owner == this, false);
        fes = fesv[fesp->idx];
        if (fesp->epoll_events.events & fd_rw_ev)
        {
            --active_count;
        }
        if (fes->non_pollable_standin)
        {
            fes->non_pollable_standin->close();
            fes->non_pollable_standin.clear();
        }
        else if (fes->epoll_events.events != 0)
        {
            /*
             * It's possible that the FES has given up ownership of the FD, so
             * we must use the passed argument, _not_ the AutoFD in the FES
             * itself.
             */
            FswAbortOnErrno(epoll_ctl(epoll_fd.get(), EPOLL_CTL_DEL, fd, NULL),
                            false);
        }
        /*
         * Swap the to-be-removed FES to the end.
         */
        const size_t old_idx = fes->idx;
        std::swap(fesv[fes->idx], fesv[fes_count - 1]);
        std::swap(fesv[old_idx]->idx, fesv[fes_count - 1]->idx);
        fes->owner = NULL;
        fes->idx = SIZE_MAX;
        --fes_count;
        /*
         * When dispatch is in progress, we defer actually deleting these
         * values, to avoid invalidating pointers being used by the dispatch
         * loop. The deletion will take place once dispatch completes.
         */
        if (!dispatch_in_progress)
        {
            fesv.pop_back();
        }
        return true;
    }
} /* end namespace Drone */