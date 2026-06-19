/**
 * @author Stefan Moluf
 * @date   03/02/11
 */
#ifndef FD_BAG_H
#define FD_BAG_H
#include "src/bullwinkle/all/FdEventSink.h"
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/core/sac.h"
#include <sys/epoll.h>
#include <vector>
/**
 * Sac if the select response code is not select_success. Does not
 * compile out.
 *
 * @param x An expression which returns a select_code_t.
 */
#define SacOnSelectError(x) SacIfNeq((x), Drone::select_success)
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_success. Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_success.
 */
#define SacAbortOnSelectError(x, retval)                                       \
    if (SacIfNeq((x), Drone::select_success))                                 \
    {                                                                          \
        return (retval);                                                       \
    }
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_error. Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_error.
 */
#define SacAbortIfNotSelectError(x, retval)                                    \
    if (SacIfNeq((x), Drone::select_error))                                   \
    {                                                                          \
        return (retval);                                                       \
    }
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_wouldblock. Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_wouldblock.
 */
#define SacAbortIfNotSelectBlock(x, retval)                                    \
    if (SacIfNeq((x), Drone::select_wouldblock))                              \
    {                                                                          \
        return (retval);                                                       \
    }
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_success. Silences all debug output of
 * the expression \a (x). Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_success.
 */
#define SacAbortSilentOnSelectError(x, retval)                                 \
    {                                                                          \
        dbsilence(true);                                                       \
        const Drone::select_code_t sac_success = (x);                         \
        dbsilence(false);                                                      \
        if (SacIfNeq((sac_success), Drone::select_success))                   \
        {                                                                      \
            return (retval);                                                   \
        }                                                                      \
    }
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_error. Silences all debug output
 * from the expression (x). Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_error.
 */
#define SacAbortSilentIfNotSelectError(x, retval)                              \
    {                                                                          \
        dbsilence(true);                                                       \
        const Drone::select_code_t sac_success = (x);                         \
        dbsilence(false);                                                      \
        if (SacIfNeq((sac_success), Drone::select_error))                     \
        {                                                                      \
            return (retval);                                                   \
        }                                                                      \
    }
/**
 * Return from the function with the supplied retval if the select
 * response code is not select_wouldblock. Silences all debug
 * output from the expression (x). Does not compile out.
 *
 * @param x An expression which returns a select_code_t.
 *
 * @param retval The value to return from the function if \a x is not
 *               select_wouldblock.
 */
#define SacAbortSilentIfNotSelectBlock(x, retval)                              \
    {                                                                          \
        dbsilence(true);                                                       \
        const Drone::select_code_t sac_success = (x);                         \
        dbsilence(false);                                                      \
        if (SacIfNeq((sac_success), Drone::select_wouldblock))                \
        {                                                                      \
            return (retval);                                                   \
        }                                                                      \
    }
namespace Drone
{
    /**
     * A response code from one of the FdBag select_*() methods.
     */
    enum select_code_t
    {
        /**
         * select() was successful. The success code casts to boolean
         * false to catch any accidental usage of SacAbortIfNot instead
         * of SacAbortOnSelectError.
         */
        select_success = 0,
        /**
         * An error occurred when attempting select().
         */
        select_error = 1,
        /**
         * select() would block indefinitely.
         */
        select_wouldblock = 2
    };
    /**
     * A collection (or bag, if you will) of FdEventSinks with a suite of
     * methods for dispatching them.
     *
     * File descriptors are registered with the fd() method. After that, any
     * calls to the select_* method will dispatch active events on them
     * through their FdEventSinks.
     */
    class FdBag
    {
    public:
        FdBag(size_t _max_fds = 1024);
        ~FdBag();
        Handle<FdEventSink> fd(AutoFd _fd);
        Handle<FdEventSink> get_fd(const int _fd);
        bool attach_fes(Handle<FdEventSink> _fes);
        bool detach_fes(Handle<FdEventSink> _fes);
        select_code_t select_absolute(const nano_t wakeup);
        select_code_t select_absolute(const nano_t wakeup, nano_t &time_used);
        select_code_t select_absolute(const nano_t wakeup,
                                      const uint fd_event_mask,
                                      nano_t &time_used);
        select_code_t select_relative(const nano_t timeout);
        select_code_t select_relative(const nano_t timeout, nano_t &time_used);
        select_code_t select_once();
        select_code_t select_until_timeout(const nano_t timeout);
        select_code_t select_until_timeout(const nano_t timeout,
                                           nano_t &time_used);
        select_code_t select_until_reads(const nano_t timeout,
                                         const uint reads);
        select_code_t select_until_writes(const nano_t timeout,
                                          const uint writes);
        uint get_event_count() const;
        uint get_read_event_count() const;
        uint get_write_event_count() const;
        uint get_except_event_count() const;
        int get_fd_bound() const;
        select_code_t raw_epoll(const nano_t wakeup,
                                struct epoll_event *events_out,
                                size_t max_events_out,
                                size_t &events_out_count);
        bool dispatch_epoll(struct epoll_event *events, size_t count,
                            uint fd_event_mask);
        template <size_t N>
        static select_code_t multi_select_absolute(FdBag *(&fdbags)[N],
                                                   const nano_t wakeup,
                                                   size_t &events_dispatched);
        static select_code_t multi_select_absolute(FdBag **fdbags,
                                                   size_t fdbag_count,
                                                   nano_t wakeup,
                                                   size_t &events_dispatched);

    private:
        friend class FdEventSink;
        bool update_epoll(FdEventSink &fes);
        bool remove_fes(int fd, FdEventSink *fesp, Handle<FdEventSink> &fes);
        /**
         * The maximum number of FDs supported.
         *
         * @note: The is the maximum size the FdBag may grow to.  The maximum
         *        fd _value_ used within the program may be higher than this.
         */
        const size_t max_fds;
        /**
         * The epoll file descriptor.
         */
        AutoFd epoll_fd;
        /**
         * List of FdEventSink objects which we own and will select on.
         */
        std::vector<Handle<FdEventSink>> fesv;
        /**
         * Scratch space for events returned from epoll_wait.
         */
        std::vector<struct epoll_event> epoll_scratch;
        /**
         * Number of attached FdEventSinks
         */
        size_t fes_count;
        /**
         * Number of active (waiting on read/write events) FdEventSinks
         */
        size_t active_count;
        /**
         * Number of event dispatches.
         */
        uint event_count;
        /**
         * Number of read event dispatches since the last select() call.
         */
        uint read_event_count;
        /**
         * Number of write event dispatches since the last select() call.
         */
        uint write_event_count;
        /**
         * Number of except event dispatches since the last select() call.
         */
        uint except_event_count;
        /**
         * Return in the case that fd() receives an invalid fd.
         */
        Handle<FdEventSink> error_fes;
        /**
         * True when dispatch is in progress, used to defer deletion of
         * FdEventSink objects.
         */
        bool dispatch_in_progress;
        /**
         * Largest fd we have seen.
         */
        int fd_bound;

    private:
        SX_DISALLOW_COPY_AND_ASSIGN(FdBag);
    };
    /**
     * Select on the file descriptors from multiple FdBags
     * simultaneously. This is the templated version that works if you know
     * the number of FdBags at compile time -- basically, a convience wrapper
     * for the non-templated version.
     *
     * @tparam N Number of FdBags to select on.
     *
     * @param fdbags Array of pointers to FdBags we should gather file
     *               descriptors from.
     * @param wakeup Wakeup at this absolute system time.
     * @param[out] events_dispatched Returns the number of events dispatched.
     *               Returns 0 if no events were triggered.
     *
     * @return Select response code.
     */
    template <size_t N>
    select_code_t FdBag::multi_select_absolute(FdBag *(&fdbags)[N],
                                               const nano_t wakeup,
                                               size_t &events_dispatched)
    {
        return multi_select_absolute(fdbags, N, wakeup, events_dispatched);
    }
} /* end namespace Drone */
#endif /* FD_BAG_H */