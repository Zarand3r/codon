/**
 * @author Stefan Moluf
 * @date   01/24/11
 */
#include "src/bullwinkle/all/io/FdDgramChannel.h"
#include "src/bullwinkle/all/AutoReset.h"
#include "src/bullwinkle/all/ReturnAccumulatorLatchFalse.h"
#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/net.h"
#include <unistd.h>
namespace Drone
{
    namespace
    {
        /**
         * Header placed before all datagrams when attached to a socket fd in
         * order to permit the storage of sender addresses along with the
         * data.
         */
        struct sockhead_t
        {
            /**
             * Length of the sockaddr.
             */
            socklen_t length;
            /**
             * Sockaddr struct which is guaranteed to be large enough for
             * all address family domains.
             */
            union safe_sockaddr_t
            {
                /**
                 * Standard sockaddr.
                 */
                sockaddr addr;
                /**
                 * AF_INET sockaddr.
                 */
                sockaddr_in in;
                /**
                 * AF_INET6 sockaddr.
                 */
                sockaddr_in6 in6;
                /**
                 * AF_UNIX sockaddr.
                 */
                sockaddr_un un;
            } addr;
        };
        /**
         * Calculate the real length of a datagram from its body. Accounts
         * for all framing data.
         *
         * @param dgram_len Datagram body length.
         *
         * @return Full datagram length.
         */
        size_t real_len(const size_t dgram_len)
        {
            return dgram_len + sizeof(sockhead_t);
        }
    } // namespace
    /**
     * Construct an FdDgramChannel.
     */
    FdDgramChannel::FdDgramChannel()
        : /*
           * By default, we reserve space for two fully-sized IP datagrams
           * with a typical MTU of 1500, plus one to detect truncation. This
           * is just one of the larger common use cases and is seen as a
           * sane default. We reserve space for two by default so that a
           * nonblocking fd can read multiple large datagrams per wakeup.
           */
          firewall_signal(), write_error_signal(), input(2, real_len(1500)),
          output(2, real_len(1501)), max_input_dgram_len(1500),
          max_output_dgram_len(1500), can_read_flag(false),
          can_write_flag(false), fes(), is_closed_flag(true),
          is_nonblocking_flag(false), fd_type(dgram_fd_none),
          handling_write(false), is_low_latency(false),
          ignore_write_errors(false), keep_only_last_dgram(false), send_flags(0)
    {}
    /**
     * Construct an FdDgramChannel.
     *
     * @param min_num_dgrams Reserve space for at least this many datagrams of
     *                       size \a _max_dgram_len
     * @param _max_dgram_len Reserve space for at least \a min_num_dgrams of
     *                       this size.
     */
    FdDgramChannel::FdDgramChannel(const size_t min_num_dgrams,
                                   const size_t max_dgram_len)
        : /*
           * Reserve an additional byte so that we can detect truncated
           * datagrams.
           */
          input(min_num_dgrams, real_len(max_dgram_len)),
          output(min_num_dgrams, real_len(max_dgram_len + 1)),
          max_input_dgram_len(max_dgram_len),
          max_output_dgram_len(max_dgram_len), can_read_flag(false),
          can_write_flag(false), fes(), is_closed_flag(true),
          is_nonblocking_flag(false), fd_type(dgram_fd_none),
          handling_write(false), is_low_latency(false),
          ignore_write_errors(false), keep_only_last_dgram(false), send_flags(0)
    {}
    /**
     * Construct an FdDgramChannel.
     *
     * @param min_num_input_dgrams Reserve space for at least this many input
     *                             datagrams of size \a max_input_dgram_len
     * @param _max_input_dgram_len Reserve space for at least \a
     *                             min_num_input_dgrams of this size.
     * @param min_num_output_dgrams Reserve space for at least this many output
     *                              datagrams of size \a max_output_dgram_len
     * @param _max_output_dgram_len Reserve space for at least \a
     *                              min_num_output_dgrams of this size.
     */
    FdDgramChannel::FdDgramChannel(const size_t min_num_input_dgrams,
                                   const size_t _max_input_dgram_len,
                                   const size_t min_num_output_dgrams,
                                   const size_t _max_output_dgram_len)
        : /*
           * Reserve an additional byte so that we can detect truncated
           * datagrams.
           */
          input(min_num_input_dgrams, real_len(_max_input_dgram_len)),
          output(min_num_output_dgrams, real_len(_max_output_dgram_len + 1)),
          max_input_dgram_len(_max_input_dgram_len),
          max_output_dgram_len(_max_output_dgram_len), can_read_flag(false),
          can_write_flag(false), fes(), is_closed_flag(true),
          is_nonblocking_flag(false), fd_type(dgram_fd_none),
          handling_write(false), is_low_latency(false),
          ignore_write_errors(false), keep_only_last_dgram(false), send_flags(0)
    {}
    /**
     * FdDgramChannel destructor.
     */
    FdDgramChannel::~FdDgramChannel() { close(); }
    /**
     * Assign a new datagram-oriented file descriptor to this channel.
     *
     * @note FdDgramChannel has no mechanism to verify that the file
     * descriptor is actually datagram-oriented, so use with care.
     *
     * @param _fes Assign this file descriptor.
     * @param _fd_type Type of this file descriptor.
     * @param is_nonblocking True if this file descriptor will never
     *                       block. Nonblocking file descriptors can
     *                       be safely read from and written to multiple
     *                       times per select event.
     * @param mode Set the read/write mode. Use one of "r", "w", "rw".
     *
     * @return True on success.
     */
    bool FdDgramChannel::assign_fd(Handle<FdEventSink> _fes,
                                   const dgram_fd_type_t _fd_type,
                                   const bool is_nonblocking,
                                   const std::string &mode)
    {
        FswAbortIfNot(_fes, false);
        FswAbortIf(_fes->is_closed(), false);
        FswAbortIf(fes, false);
        /*
         * Verify fd type.
         */
        FswAbortIfEqInt(_fd_type, dgram_fd_none, false);
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
                    make_slot(*this, &FdDgramChannel::handle_fd_read)),
                false);
        }
        if (can_write_flag)
        {
            FswAbortIfNot(
                _fes->add_events(
                    fd_write_ev,
                    make_slot(*this, &FdDgramChannel::handle_fd_write)),
                false);
        }
        FswAbortIfNot(_fes->add_events(
                          fd_close_ev,
                          make_slot(*this, &FdDgramChannel::handle_fd_close)),
                      false);
        /*
         * Reset flags.
         */
        fd_type = _fd_type;
        is_nonblocking_flag = is_nonblocking;
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
    bool FdDgramChannel::get_fd(Handle<FdEventSink> &_fes) const
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
     * Get the sockaddr of the first datagram in the channel.
     *
     * @param[out] addr Writes sockaddr to this pointer.
     * @param[in,out] addr_len Indicates the size of \a addr. Returns the
     *                         length of the copied data.
     *
     * @return True if sockaddrs are being recorded, a datagram is
     *         in the channel, and \a addr_len is sufficient to hold
     *         the sockaddr for this datagram.
     */
    bool FdDgramChannel::get_address(sockaddr *const addr,
                                     socklen_t *const addr_len) const
    {
        FswAbortIfNot(dgrams_avail(), false);
        FswAbortIfNeqInt(dgram_fd_uconn_socket, fd_type, false);
        const B2c dgram = output.get_b2c();
        FswAbortIfNot(dgram.buf(), false);
        FswAssert(dgram.len() >= sizeof(sockhead_t));
        sockhead_t head = {};
        memcpy(&head, dgram.buf(), sizeof(head));
        /*
         * Copy our sockaddr. Return false if the user can't fit
         * our sockaddr.
         */
        FswAbortIfNotOpInt(*addr_len, >=, head.length, false);
        memcpy(addr, &head.addr, head.length);
        *addr_len = head.length;
        return true;
    }
    /**
     * Returns true if the fd was opened with read mode set.
     *
     * @return true if the fd was opened with read mode set.
     */
    bool FdDgramChannel::can_read() const { return can_read_flag; }
    /**
     * Returns true if the fd was opened with write mode set.
     *
     * @return true if the fd was opened with write mode set.
     */
    bool FdDgramChannel::can_write() const { return can_write_flag; }
    /*
     * See DgramChannel.
     */
    bool FdDgramChannel::clear()
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
     * See DgramChannel.
     */
    bool FdDgramChannel::is_closed() const { return is_closed_flag || !fes; }
    /*
     * See DgramChannel.
     */
    bool FdDgramChannel::is_empty() const { return output.empty(); }
    /**
     * Returns true if the channel is closed and both input and output
     * buffers are empty.
     *
     * @return True if the channel is closed and both input and output
     *         buffers are empty.
     */
    bool FdDgramChannel::is_drained() const
    {
        return is_closed() && input.empty() && output.empty();
    }
    /**
     * Returns the space available to write, up to the size of a full
     * datagram. Will return 0 if there isn't enough space for internal
     * headers.
     *
     * @return Space available to write.
     */
    size_t FdDgramChannel::space_left() const
    {
        if (is_closed())
            return 0;
        if (!can_write_flag)
            return 0;
        if (is_low_latency)
        {
            /*
             * Never allow more than one datagram in the buffer when
             * low-latency because we can't guarantee the second datagram will
             * be writable immediately.
             */
            if (!input.empty())
            {
                return 0;
            }
            /*
             * There are only two cases where we are sure a datagram can be
             * written to the fd immediately without blocking:
             *
             *   - handle_fd_write() was dispatched and no datagram was
             *       available, causing fd_write_ev to be removed from the fes.
             *   - We are in the middle of a call to handle_fd_write().
             *
             * This if-statement detects those two cases.
             */
            if ((!fes || (fes->get_events() & fd_write_ev)) && !handling_write)
            {
                return 0;
            }
        }
        const size_t space = input.space_left();
        if (space < sizeof(sockhead_t))
            return 0;
        return std::min(space - sizeof(sockhead_t), max_input_dgram_len);
    }
    /*
     * See DgramChannel.
     */
    DataFrame &FdDgramChannel::get_dataframe(const size_t request_len)
    {
        if (space_left() == 0)
        {
            return DataFrame::null();
        }
        const size_t real_request = std::min(request_len, max_input_dgram_len);
        DataFrame &df = input.get_dataframe(real_request + sizeof(sockhead_t));
        if (!df.mask(sizeof(sockhead_t)))
            return DataFrame::null();
        return df;
    }
    /**
     * Finalize a write to the buffer. Must be called after writing into
     * the DataFrame object returned by get_dataframe(), or all written data
     * will be discarded.
     *
     * May not be used by fd types which require sockaddrs.
     *
     * @param frame Commit the contents of this object. Must be the same
     *              object that was returned from get_dataframe().
     *
     * @return True on success.
     */
    bool FdDgramChannel::commit_dataframe(DataFrame &frame)
    {
        FswAbortIf(is_closed(), false);
        if (dgram_fd_uconn_socket == fd_type)
        {
            FswPrefix();
            dbstring(": A destination is required for unconnected sockets.\n");
            return false;
        }
        FswAbortIfNot(frame.body.get_data_len() <= max_input_dgram_len, false);
        /*
         * Write zeroes into the header.
         */
        FswAbortIfNot(frame.unmask(sizeof(sockhead_t)), false);
        frame.header.fill_with_zeros(frame.header.space_left());
        FswAbortIfNot(channel_commit_dataframe(frame), false);
        return true;
    }
    /**
     * Finalize a write to the buffer. Must be called after writing into
     * the DataFrame object returned by get_dataframe(), or all written data
     * will be discarded.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param frame Commit the contents of this object. Must be the same
     *              object that was returned from get_dataframe().
     * @param addr Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return True on success.
     */
    bool FdDgramChannel::commit_dataframe_to(DataFrame &frame,
                                             const sockaddr *const addr,
                                             const socklen_t addr_len)
    {
        FswAbortIf(is_closed(), false);
        FswAbortIfNot(addr, false);
        FswAbortIf(0 == addr_len, false);
        FswAbortIfNotOpInt(sizeof(sockhead_t::safe_sockaddr_t), >=, addr_len,
                           false);
        if (dgram_fd_uconn_socket != fd_type)
        {
            FswPrefix();
            dbstring(": A destination is forbidden for connected sockets "
                     "and non-sockets.\n");
            return false;
        }
        FswAbortIfNotOpInt(frame.body.get_data_len(), <=, max_input_dgram_len,
                           false);
        /*
         * Write the sockaddr into the header.
         */
        FswAbortIfNot(frame.unmask(sizeof(sockhead_t)), false);
        sockhead_t head;
        head.length = addr_len;
        memcpy(&head.addr, addr, addr_len);
        FswAbortIfNot(frame.header.push_back((char *)&head, sizeof(head)),
                      false);
        FswAbortIfNot(channel_commit_dataframe(frame), false);
        return true;
    }
    /**
     * Write a datagram into the channel.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data Pointer to the datagram.
     * @param data_len Length of the datagram.
     * @param addr Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool FdDgramChannel::write_to(const char *data, const size_t data_len,
                                  const sockaddr *const addr,
                                  const socklen_t addr_len)
    {
        DataFrame &df = get_dataframe(data_len);
        if (df.body.space_left() < data_len)
            return false;
        FswAbortIfNot(df.body.push_back(data, data_len), false);
        FswAbortIfNot(commit_dataframe_to(df, addr, addr_len), false);
        return true;
    }
    /**
     * Write a datagram into the channel.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data Pointer to the datagram.
     * @param addr Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool FdDgramChannel::write_to(const B2c &data, const sockaddr *const addr,
                                  const socklen_t addr_len)
    {
        FswAbortIfNot(write_to(data.buf(), data.len(), addr, addr_len), false);
        return true;
    }
    /**
     * Returns the number of datagrams present in the output channel.
     *
     * @return The number of datagrams present in the output channel.
     */
    uint FdDgramChannel::dgrams_avail() const { return output.dgrams_avail(); }
    /*
     * See DgramChannel.
     */
    B2c FdDgramChannel::peek_dgram() const
    {
        const B2c temp = output.get_b2c();
        if (NULL == temp.buf() || 0 == temp.len())
            return B2c(NULL, 0);
        FswAssert(temp.len() >= sizeof(sockhead_t));
        return B2c(temp.buf() + sizeof(sockhead_t),
                   temp.len() - sizeof(sockhead_t));
    }
    /**
     * Clear connected signals.
     *
     * @return True on success.
     */
    bool FdDgramChannel::clear_signals()
    {
        firewall_signal.clear();
        FswAbortIfNot(DgramChannel::clear_signals(), false);
        return true;
    }
    /**
     * Pop the first datagram from the channel.
     *
     * Unlike a typical DgramChannel, this will not produce a write
     * signal, since the write channel has not opened up any space.
     *
     * @return True if a datagram was present.
     */
    bool FdDgramChannel::pop_dgram()
    {
        FswAbortIf(is_empty(), false);
        FswAbortIfNot(channel_pop_dgram(), false);
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
    /**
     * Commits the DataFrame to the input buffer and begins selecting
     * on write events.
     *
     * @param frame Commits this DataFrame.
     *
     * @return True on success.
     */
    bool FdDgramChannel::channel_commit_dataframe(DataFrame &frame)
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
    /*
     * See DgramChannel.
     */
    bool FdDgramChannel::channel_close()
    {
        is_closed_flag = true;
        /*
         * If we don't have a file descriptor or our file
         * descriptor is closed, we can't drain the input buffer
         * into it. Clear it.
         */
        if (!fes)
        {
            FswAbortIfNot(input.clear(), false);
            return true;
        }
        else if (fes->is_closed())
        {
            /*
             * Don't return just yet, we still need to clear
             * the fes.
             */
            FswAbortIfNot(input.clear(), false);
        }
        /*
         * Stop listening for read events. We only keep the fd open
         * to drain the input buffer.
         */
        FswAbortIfNot(fes->remove_events(fd_read_ev), false);
        /*
         * If there is still data left to flush to the descriptor, we have
         * to wait.
         */
        if (!input.empty())
            return true;
        /*
         * Close the fes once we've drained the input buffer.
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
     * Clears the output channel of all datagrams.
     *
     * @return True on success.
     */
    bool FdDgramChannel::channel_clear()
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
     * See DgramChannel.
     */
    bool FdDgramChannel::channel_pop_dgram()
    {
        FswAbortIfNot(output.pop_dgram(), false);
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
    bool FdDgramChannel::handle_fd_read(FdEventSink &_fes, FdEvent &fev)
    {
        FswAbortIfNot(fes == &_fes, false);
        FswAbortIfNot(fes->get_fd() == fev.fd, false);
        /*
         * We reserve one more byte than we need in order to detect
         * truncated datagrams.
         */
        const size_t reserve_len = real_len(max_output_dgram_len) + 1;
        size_t dgrams_read = 0;
        /*
         * Because datagrams will be silently truncated if we don't
         * provide large enough buffers, we must wait until we have
         * enough space to store a maximally-sized datagram. Ask
         * clients to read data if we don't have enough.
         *
         * Note that we have to do this outside the loop to ensure that a
         * user who is continually consuming datagrams and emptying the
         * read buffer doesn't cause us to read arbitrarily large amounts
         * of data from the kernel in a single cycle.
         */
        if (output.space_left() < reserve_len)
        {
            FswAbortIfNot(signal_read(), false);
        }
        /*
         * Read at least once, but only keep reading until exhaustion if
         * we've been explicitly told that the file descriptor respects
         * non-blocking semantics. Otherwise, we might read the first
         * datagram, then attempt to read the second, and then block for
         * an indefinite period.
         */
        do
        {
            DataFrame &df = output.get_dataframe(reserve_len);
            /*
             * If we still don't have enough, switch off read events
             * until someone reads data from us.
             */
            if (df.body.space_left() < reserve_len)
            {
                FswAbortIfNot(fes->remove_events(fd_read_ev), false);
                break;
            }
            /*
             * Calculate offsets for the header and body.
             */
            sockhead_t head = {};
            char *raw_head = df.get_raw();
            FswAbortIfNot(raw_head, false);
            char *const body = df.get_raw() + sizeof(sockhead_t);
            const size_t bodylen = df.space_left_raw() - sizeof(sockhead_t);
            ssize_t retval = -1;
            /*
             * If we have a socket fd, we can get the source address
             * along with the datagram. Otherwise we just write zero into
             * the sockhead.
             */
            if (dgram_fd_conn_socket == fd_type ||
                dgram_fd_uconn_socket == fd_type)
            {
                /*
                 * Write sockaddr length into the buffer.
                 */
                head.length = sizeof(head.addr);
                /*
                 * Read from the fd.
                 */
                retval = fsw_recvfrom(fev.fd, body, bodylen, 0,
                                     (sockaddr *)&head.addr, &head.length);
                /*
                 * If we were returned a sockaddr which is larger than we were
                 * actually able to record, our addrlen is now longer than the
                 * actual addr portion of the buffer. We need to clamp it down
                 * or we will read past the end of the sockhead_t structure.
                 */
                if ((size_t)head.length > sizeof(head.addr))
                {
                    head.length = sizeof(head.addr);
                }
                /*
                 * Drop this dgram if it is not allowed by the firewall.
                 *
                 * Ignore sockaddr_un addresses (local fd) since local traffic
                 * is never blocked.
                 */
                if (!firewall_signal.empty() && head.length <= sizeof(sockaddr))
                {
                    using accumulator_t =
                        ReturnAccumulatorLatchFalse<const sockaddr &,
                                                    const socklen_t &>;
                    if (!firewall_signal.emit<accumulator_t>(head.addr.addr,
                                                             head.length))
                    {
                        continue;
                    }
                }
            }
            else if (dgram_fd_non_socket == fd_type ||
                     dgram_fd_device_file == fd_type || dgram_fd_tun == fd_type)
            {
                /*
                 * Read from the fd.
                 */
                retval = ::read(fev.fd, body, bodylen);
            }
            else
            {
                FswPrefix();
                dbnprintf(100, ": unsupported socket type %d\n", (int)fd_type);
                return false;
            }
            memcpy(raw_head, &head, sizeof(head));
            if (retval < 0)
            {
                if (is_fd_broken(retval, "FdDgramChannel read error"))
                {
                    FswPrefix();
                    dbnprintf(100, ": Read failed: size=%zu, fd=%d\n", bodylen,
                              fev.fd);
                    return false;
                }
                break;
            }
            /*
             * If the datagram was longer than the maximum length, it is
             * invalid, probably truncated, and we discard it.
             */
            if ((size_t)retval > max_output_dgram_len)
                continue;
            /*
             * If we get a zero read with a non-socket fd, then we have
             * hit EOF and return false to close the fd.
             */
            if (dgram_fd_non_socket == fd_type || dgram_fd_tun == fd_type)
            {
                if (0 == retval)
                    return false;
            }
            /*
             * If this is a socket, all nonnegative lengths (even 0) less
             * than the maximum are legitimate. Commit the bytes.
             */
            FswAbortIfNot(df.commit_raw(sizeof(sockhead_t) + retval), false);
            FswAbortIfNot(output.commit_dataframe(df), false);
            dgrams_read++;
            /*
             * If configured to only keep the last dgram, pop any other dgrams.
             */
            if (keep_only_last_dgram && output.dgrams_avail() > 1)
            {
                FswAbortIfNot(output.pop_dgram(), false);
            }
        }
        /*
         * Keep reading if this fd is nonblocking.
         */
        while (is_nonblocking_flag);
        /*
         * Signal new data to clients.
         */
        if (dgrams_read)
        {
            FswAbortIfNot(signal_read(), false);
        }
        /*
         * Return true in the blocking fd case.
         */
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
    bool FdDgramChannel::handle_fd_write(FdEventSink &_fes, FdEvent &fev)
    {
        /*
         * Set handling_write to true for the duration of this method
         * and automatically reset it back to false wherever we return.
         */
        FswAbortIf(handling_write, false);
        AutoReset<bool> resetter(handling_write, true);
        FswAbortIfNot(fes == &_fes, false);
        FswAbortIfNot(fes->get_fd() == fev.fd, false);
        /*
         * If we are empty, ask for more datagrams.
         */
        if (!is_closed() && input.empty())
        {
            FswAbortIfNot(signal_write(), false);
        }
        do
        {
            const uint orig_dgrams_avail = input.dgrams_avail();
            if (input.dgrams_avail() > 0)
            {
                const B2c data = input.get_b2c();
                /*
                 * We must have at least a header. If not, something went
                 * wrong in the read routine.
                 */
                FswAssert(data.len() >= sizeof(sockhead_t));
                /*
                 * Compute head and body offsets.
                 */
                sockhead_t head = {};
                memcpy(&head, data.buf(), sizeof(sockhead_t));
                const char *const body = data.buf() + sizeof(sockhead_t);
                const size_t bodylen = data.len() - sizeof(sockhead_t);
                ssize_t retval = -1;
                if (dgram_fd_uconn_socket == fd_type)
                {
                    /*
                     * Write the datagram to the fd.
                     */
                    retval = fsw_sendto(fev.fd, body, bodylen, send_flags,
                                       (sockaddr *)&head.addr, head.length);
                }
                else if (dgram_fd_conn_socket == fd_type ||
                         dgram_fd_non_socket == fd_type ||
                         dgram_fd_device_file == fd_type ||
                         dgram_fd_tun == fd_type)
                {
                    /*
                     * Write the datagram to the fd.
                     */
                    retval = ::write(fev.fd, body, bodylen);
                }
                else
                {
                    FswPrefix();
                    dbstring(": unsupported socket type!\n");
                    return false;
                }
                /*
                 * Handle a very specific case with Tun devices - if a
                 * packet with an invalid IP header is written to the
                 * Tun it will return EINVAL. While it is a serious error
                 * indicating that invalid data is being passed in the
                 * system, this is not a problem with the Tun device or
                 * the channel so should not cause the the fd to be torn
                 * down. Instead, we eat the error and continue. The
                 * Tun network interface increments its dropped packet
                 * counter when this occurs, so this error condition
                 * is visible from telemetry.
                 */
                if (dgram_fd_tun == fd_type && retval < 0 && EINVAL == errno)
                {
                    retval = bodylen;
                }
                /*
                 * Generic error case -- if there is an error handler,
                 * call it and see if the user wants us to ignore
                 * this.
                 */
                if (!write_error_signal.empty() && retval < 0)
                {
                    bool ignore_error =
                        write_error_signal.emit(B2c(body, bodylen), errno);
                    if (ignore_error)
                    {
                        retval = bodylen;
                    }
                }
                if (retval < 0)
                {
                    if (ignore_write_errors &&
                        ((errno == EACCES) || (errno == ECONNREFUSED) ||
                         (errno == ENOENT)))
                    {
                        return true;
                    }
                    if (is_fd_broken(retval, "FdDgramChannel write error"))
                    {
                        if (dgram_fd_uconn_socket == fd_type &&
                            head.length >= sizeof(sockaddr_in) &&
                            head.addr.in.sin_family == AF_INET)
                        {
                            /*
                             * Note that inet_ntoa() uses a static buffer
                             */
                            FswPrefix();
                            dbnprintf(100,
                                      ": Write failed: size=%zu, fd=%d, "
                                      "dest=%s:%d\n",
                                      bodylen, fev.fd,
                                      inet_ntoa(head.addr.in.sin_addr),
                                      ntohs(head.addr.in.sin_port));
                        }
                        else if (dgram_fd_uconn_socket == fd_type &&
                                 head.length >= sizeof(sockaddr_in6) &&
                                 head.addr.in6.sin6_family == AF_INET6)
                        {
                            FswPrefix();
                            /*
                             * Note that inet_ntop() requires passing in a
                             * buffer, and can return null.
                             */
                            char dest_storage[INET6_ADDRSTRLEN] = {};
                            const char *dest =
                                inet_ntop(AF_INET6, &head.addr.in6.sin6_addr,
                                          dest_storage, INET6_ADDRSTRLEN);
                            dbnprintf(200,
                                      ": Write failed: size=%zu, fd=%d, "
                                      "dest=[%s]:%d\n",
                                      bodylen, fev.fd, dest ? dest : "",
                                      ntohs(head.addr.in6.sin6_port));
                        }
                        else
                        {
                            FswPrefix();
                            dbnprintf(100, ": Write failed: size=%zu, fd=%d\n",
                                      bodylen, fev.fd);
                        }
                        return false;
                    }
                    return true;
                }
                FswAbortIfNeq(retval, bodylen, false);
                FswAbortIfNot(input.pop_dgram(), false);
            }
            if (!is_closed())
            {
                /*
                 * If we freed some buffer space, ask for more datagrams
                 * unless we are low-latency in which case we only wanted one
                 * anyways.
                 */
                if (input.dgrams_avail() < orig_dgrams_avail && !is_low_latency)
                {
                    FswAbortIfNot(signal_write(), false);
                }
            }
            /*
             * Check if we're closed again. signal_write() may have drained us,
             * which would invalidate fes.
             */
            if (!is_closed())
            {
                /*
                 * If no datagrams arrived, switch off write events until
                 * we get more data. If low-latency, it's orig_dgrams_avail
                 * that matter because we never asked for more.
                 */
                if (orig_dgrams_avail == 0 ||
                    (!is_low_latency && input.empty()))
                {
                    FswAbortIfNot(fes->remove_events(fd_write_ev), false);
                    break;
                }
            }
            /*
             * If we are now drained, return false to close the
             * file descriptor.
             */
            else if (0 == input.dgrams_avail())
            {
                return false;
            }
        }
        /*
         * Keep writing if this fd is nonblocking and we are not low-latency.
         */
        while (is_nonblocking_flag && !is_low_latency);
        return true;
    }
    /**
     * Clears all datagrams that were queued up by write(), write_to(),
     * commit_dataframe(), etc but have not yet been written to the file
     * descriptor.
     *
     * Before calling this function, be sure that the remote side can handle
     * having some of its data being chopped from the middle of the data
     * stream.
     *
     * @return True on success.
     */
    bool FdDgramChannel::clear_unwritten()
    {
        if (!input.empty())
        {
            FswAbortIfNot(input.clear(), false);
            if (is_closed())
            {
                /*
                 * Call channel_close to close the fd and finish cleaning up.
                 */
                FswAbortIfNot(channel_close(), false);
            }
            else if (space_left())
            {
                /*
                 * Request more data.
                 */
                FswAbortIfNot(signal_write(), false);
                if (input.empty())
                {
                    /*
                     * Switch off write events until we get more data.
                     */
                    FswAbortIfNot(fes->remove_events(fd_write_ev), false);
                }
            }
        }
        return true;
    }
    /**
     * Returns the number of datagrams available to be written to the file
     * descriptor. This is very different from dgrams_avail(), which returns
     * the number of datagrams available for reading. In other words, this
     * number goes up when write() is called.
     *
     * @return Number of datagrams added to this Channel by write() but not
     *         yet written out to the file descriptor.
     */
    uint FdDgramChannel::dgrams_unwritten() const
    {
        return input.dgrams_avail();
    }
    /**
     * Handle a file descriptor close event.
     *
     * @param _fes FdEventSink that generated the event.
     * @param fev The FdEvent.
     *
     * @return True on success.
     */
    bool FdDgramChannel::handle_fd_close(FdEventSink &_fes, FdEvent &fev)
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