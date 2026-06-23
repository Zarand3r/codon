/**
 * @author Stefan Moluf
 * @date   01/24/11
 */
#include "src/bullwinkle/all/io/FdStreamChannel.h"
#include <unistd.h>
namespace Drone
{
    /**
     * Construct an FdStreamChannel.
     */
    FdStreamChannel::FdStreamChannel()
        : /*
           * By default, we reserve space a fully-sized IP datagram with a
           * typical MTU of 1500. This is just one of the larger common use
           * cases and is seen as a sane default.
           */
          input(1500), output(1500), can_read_flag(false),
          can_write_flag(false), fes(), is_closed_flag(true)
    {}
    /**
     * Construct an FdStreamChannel.
     *
     * @param size Creates input and output buffers of this size.
     */
    FdStreamChannel::FdStreamChannel(const size_t size)
        : input(size), output(size), can_read_flag(false),
          can_write_flag(false), fes(), is_closed_flag(true)
    {}
    /**
     * FdStreamChannel destructor.
     */
    FdStreamChannel::~FdStreamChannel() { close(); }
    /**
     * Assign a new stream-oriented file descriptor to this channel.
     *
     * @note FdStreamChannel has no mechanism to verify that the file
     * descriptor is actually stream-oriented, so use with care.
     *
     * @param _fes Assign this file descriptor.
     * @param mode Set the read/write mode. Use one of "r", "w", "rw".
     *
     * @return True on success.
     */
    bool FdStreamChannel::assign_fd(Handle<FdEventSink> _fes,
                                    const std::string &mode)
    {
        FswAbortIfNot(_fes, false);
        FswAbortIf(_fes->is_closed(), false);
        FswAbortIf(fes, false);
        /*
         * Verify mode string.
         */
        if (mode != "r" && mode != "w" && mode != "rw")
        {
            FswPrefix();
            dbnprintf(100, ": Allowed modes are 'r', 'w', 'rw', got '%s'\n",
                      mode.c_str());
            return false;
        }
        /*
         * Set permissions flags.
         */
        can_read_flag = (std::string::npos != mode.find("r"));
        can_write_flag = (std::string::npos != mode.find("w"));
        /*
         * Attach signals.
         */
        if (can_read_flag)
        {
            FswAbortIfNot(
                _fes->add_events(
                    fd_read_ev,
                    make_slot(*this, &FdStreamChannel::handle_fd_read)),
                false);
        }
        if (can_write_flag)
        {
            FswAbortIfNot(
                _fes->add_events(
                    fd_write_ev,
                    make_slot(*this, &FdStreamChannel::handle_fd_write)),
                false);
        }
        FswAbortIfNot(_fes->add_events(
                          fd_close_ev,
                          make_slot(*this, &FdStreamChannel::handle_fd_close)),
                      false);
        fes = _fes;
        is_closed_flag = false;
        return true;
    }
    /**
     * Access the FdEventSink.
     *
     * @param[out] _fes Returns the FdEventSink.
     *
     * @return True on success.
     */
    bool FdStreamChannel::get_fd(Handle<FdEventSink> &_fes) const
    {
        if (!fes)
        {
            _fes.clear();
            return false;
        }
        _fes = fes;
        return true;
    }
    /**
     * Returns true if the fd was opened with read mode set.
     *
     * @return true if the fd was opened with read mode set.
     */
    bool FdStreamChannel::can_read() const { return can_read_flag; }
    /**
     * Returns true if the fd was opened with write mode set.
     *
     * @return true if the fd was opened with write mode set.
     */
    bool FdStreamChannel::can_write() const { return can_write_flag; }
    /*
     * See StreamChannel.
     */
    bool FdStreamChannel::clear()
    {
        if (is_empty())
            return true;
        FswAbortIfNot(channel_clear(), false);
        /*
         * If we are closed and have just drained the channel, emit
         * a close signal.
         */
        if (is_drained())
        {
            /*
             * There's not a lot we can do if this fails. We've already
             * successfully closed the channel and cleared the data.
             */
            FswIfNot(signal_close());
        }
        return true;
    }
    /*
     * See StreamChannel.
     */
    bool FdStreamChannel::is_closed() const { return is_closed_flag || !fes; }
    /*
     * See StreamChannel.
     */
    bool FdStreamChannel::is_empty() const { return output.is_empty(); }
    /**
     * Returns true if the channel is closed and both input and output
     * buffers are empty.
     *
     * @return True if the channel is closed and both input and output
     *         buffers are empty.
     */
    bool FdStreamChannel::is_drained() const
    {
        return is_closed() && input.is_empty() && output.is_empty();
    }
    /*
     * See StreamChannel.
     */
    size_t FdStreamChannel::get_max_data_len() const
    {
        return output.get_max_size();
    }
    /*
     * See StreamChannel.
     */
    size_t FdStreamChannel::space_left() const
    {
        if (is_closed())
            return 0;
        if (!can_write_flag)
            return 0;
        return input.space_left();
    }
    /**
     * Return the space currently taken up by writes into the channel (i.e.,
     * the amount of data yet to be flushed to the underlying file descriptor).
     *
     * @return The space currently taken up by writes into the channel.
     */
    size_t FdStreamChannel::space_taken() const
    {
        if (is_closed())
            return 0;
        if (!can_write_flag)
            return 0;
        return input.get_data_len();
    }
    /*
     * See StreamChannel.
     */
    DataFrame &FdStreamChannel::get_dataframe(const size_t request_len)
    {
        if (is_closed())
            return DataFrame::null();
        if (!can_write_flag)
            return DataFrame::null();
        return input.get_dataframe(request_len);
    }
    /*
     * See StreamChannel.
     */
    B2c FdStreamChannel::get_data() const { return output.get_b2(); }
    /**
     * Remove bytes from the front of the channel.
     *
     * After reading bytes using get_data() and get_data_len(), you
     * can remove them (or some of them) from the channel with this
     * function.
     *
     * Unlike a typical StreamChannel, this will not produce a write
     * signal, since the write channel has not opened up any space.
     *
     * @param bytes Pop this many bytes.
     *
     * @return True if all requested bytes were removed. False on
     *         error. If there were insufficient bytes, no bytes will
     *         be removed.
     */
    bool FdStreamChannel::pop_front(const size_t bytes)
    {
        if (0 == bytes)
            return true;
        FswAbortIfNot(channel_pop_front(bytes), false);
        has_popped = true;
        /*
         * If someone removes data from our channel, switch on
         * read events to start consuming more from the fd.
         */
        if (!is_closed() && can_read_flag)
        {
            FswAbortIfNot(fes->add_events(fd_read_ev), false);
        }
        /*
         * If we have just drained the channel, emit a close signal.
         */
        else if (is_drained())
        {
            FswIfNot(signal_close());
        }
        return true;
    }
    /*
     * See StreamChannel.
     */
    bool FdStreamChannel::channel_pop_front(const size_t bytes)
    {
        FswAbortIfNot(output.pop_front(bytes), false);
        return true;
    }
    /**
     * Clears the output channel of all data.
     *
     * @return True on success.
     */
    bool FdStreamChannel::channel_clear()
    {
        FswAbortIfNot(output.clear(), false);
        /*
         * If someone removes data from our channel, switch on
         * read events to start consuming more from the fd.
         */
        if (!is_closed() && can_read_flag)
        {
            FswAbortIfNot(fes->add_events(fd_read_ev), false);
        }
        return true;
    }
    /*
     * See StreamChannel.
     */
    bool FdStreamChannel::channel_close()
    {
        is_closed_flag = true;
        /*
         * If we have no file descriptor, clear any queued data.
         */
        if (!fes)
        {
            FswAbortIfNot(input.clear(), false);
            return true;
        }
        else if (fes->is_closed())
        {
            FswAbortIfNot(input.clear(), false);
        }
        /*
         * Stop selecting on read events.
         */
        FswAbortIfNot(fes->remove_events(fd_read_ev), false);
        /*
         * If there is still data left to flush to the descriptor, we have
         * to wait.
         */
        if (!input.is_empty())
            return true;
        /*
         * If we're empty, then close and clear the file descriptor.
         */
        if (!fes->is_closed())
            fes->close();
        /*
         * Clear the fes so that we can have assign_fd() called again.
         *
         * Calling close() above will cause us to re-enter this function
         * through handle_fd_close(). Luckily this next step here is
         * idempotent, but if you want to add anything to this function,
         * beware of that fact.
         */
        fes.clear();
        can_read_flag = false;
        can_write_flag = false;
        return true;
    }
    /**
     * Commits the DataFrame to the input buffer and begins selecting
     * on write events.
     *
     * @param frame Commits this DataFrame.
     *
     * @return True on success.
     */
    bool FdStreamChannel::channel_commit_dataframe(DataFrame &frame)
    {
        FswAbortIfNot(can_write_flag, false);
        FswAbortIfNot(input.commit_dataframe(frame), false);
        /*
         * If someone wrote data to our channel, switch on write events
         * to start pushing it down the file descriptor.
         */
        FswAbortIfNot(fes, false);
        FswAbortIfNot(fes->add_events(fd_write_ev), false);
        return true;
    }
    /**
     * Handle a file descriptor read event. Read data from the fd into
     * the internal buffers and emit signals as necessary.
     *
     * @param _fes FdEventSink that generated the event.
     * @param fev The FdEvent.
     *
     * @return True on success. Returning false from this method
     *         will cause the FdEventSink to close.
     */
    bool FdStreamChannel::handle_fd_read(FdEventSink &_fes, FdEvent &fev)
    {
        FswAbortIfNot(fes == &_fes, false);
        FswAbortIfNot(fes->get_fd() == fev.fd, false);
        /*
         * If we have zero space left, ask the read_sig to free up
         * some data in our buffer.
         */
        if (!output.space_left())
            FswAbortIfNot(signal_read(), false);
        /*
         * If we still don't have any space, switch off read events
         * until someone pulls data from our channel.
         */
        const size_t space = output.space_left();
        if (!space)
        {
            FswAbortIfNot(fes->remove_events(fd_read_ev), false);
            return true;
        }
        /*
         * Read data from the file descriptor directly into the
         * output channel.
         */
        DataFrame &df = output.get_dataframe(space);
        const ssize_t retval =
            ::read(fev.fd, df.get_raw(), df.space_left_raw());
        if (retval < 0)
        {
            if (is_fd_broken(retval, "FdStreamChannel read error"))
            {
                return false;
            }
            return true;
        }
        /*
         * EOF. Return false to close the FdEventSink.
         */
        if (0 == retval)
            return false;
        /*
         * Commit bytes to the output channel.
         */
        FswAbortIfNot(df.commit_raw(retval), false);
        FswAbortIfNot(output.commit_dataframe(df), false);
        /*
         * Signal new data to clients.
         */
        FswAbortIfNot(signal_read(), false);
        return true;
    }
    /**
     * Handle a file descriptor write event. Write data from the internal
     * buffers into the fd and emit signals as necessary.
     *
     * @param _fes FdEventSink that generated the event.
     * @param fev The FdEvent.
     *
     * @return True on success. Returning false from this method
     *         will cause the FdEventSink to close.
     */
    bool FdStreamChannel::handle_fd_write(FdEventSink &_fes, FdEvent &fev)
    {
        FswAbortIfNot(fes == &_fes, false);
        FswAbortIfNot(fes->get_fd() == fev.fd, false);
        /*
         * If we are empty, ask for more data.
         */
        if (!is_closed() && input.is_empty())
            FswAbortIfNot(signal_write(), false);
        const B2c data = input.get_b2();
        const bool was_empty = input.is_empty();
        if (data.len() > 0)
        {
            const ssize_t retval = ::write(fev.fd, data.buf(), data.len());
            if (retval < 0)
            {
                if (is_fd_broken(retval, "FdStreamChannel write error"))
                {
                    return false;
                }
                return true;
            }
            FswAbortIfNot(input.pop_front(retval), false);
        }
        if (!is_closed())
        {
            /*
             * If we have emptied our buffer, ask for more data.
             */
            if (!was_empty && input.is_empty())
                FswAbortIfNot(signal_write(), false);
        }
        /*
         * Check if we're closed again. signal_write() may have drained us,
         * which would invalidate fes.
         */
        if (!is_closed())
        {
            /*
             * If no more arrived, switch off write events until we get
             * more data.
             */
            if (input.is_empty())
                FswAbortIfNot(fes->remove_events(fd_write_ev), false);
        }
        /*
         * If we are now drained, close the file descriptor by returning
         * false.
         */
        else if (input.is_empty())
        {
            return false;
        }
        return true;
    }
    /**
     * Handle a file descriptor close event.
     *
     * @param _fes FdEventSink that generated the event.
     * @param fev The FdEvent.
     *
     * @return True on success.
     */
    bool FdStreamChannel::handle_fd_close(FdEventSink &_fes, FdEvent &fev)
    {
        bool ret = true;
        /*
         * If the fes handle is freed behind our back, we still want
         * to emit a close signal. Thus we complain here, but do
         * not abort.
         */
        FswIfNot2(fes == &_fes, ret);
        if (fes)
            FswIfNot2(fes->get_fd() == fev.fd, ret);
        FswIfNot2(channel_close(), ret);
        if (is_drained())
            FswIfNot(signal_close());
        return ret;
    }
} /* end namespace Drone */