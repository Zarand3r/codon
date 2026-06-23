/**
 * @author Stefan Moluf
 * @date   04/18/11
 */
#include "src/bullwinkle/all/io/TcpConnection.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/sock_util.h"
namespace Drone
{
    /**
     * Construct a TcpConnection.
     *
     * No connections will begin until connect() is called.
     *
     * @param _elist Attaches a Reconnector to this EventList.
     * @param size Size of internal buffer.
     * @param _fd_bag The FdBag from which to obtain the FDs for
     *                this connection.
     */
    TcpConnection::TcpConnection(EventList &_elist, FdBag &_fd_bag,
                                 const size_t size)
        : elist(_elist), fd_bag(_fd_bag), reconnector(), fd_channel(size),
          is_closed_flag(false), has_popped(false), use_keepalive(false),
          keepalive_probe_interval(0), keepalive_retry_interval(0),
          keepalive_retry_count(0), socket_receive_buf_size(0),
          socket_send_buf_size(0)
    {
        /*
         * Enable keepalives by default.
         */
        enable_tcp_keepalive();
    }
    /**
     * TcpConnection destructor.
     */
    TcpConnection::~TcpConnection() { close(); }
    /**
     * Connect to a TCP port.
     *
     * @param host_port Hostname and (optionally) port to connect to.
     * @param default_port Connect to this port if none is specified.
     * @param tcp_no_delay Set TCP_NODELAY if true.
     *
     * @return True on success.
     */
    bool TcpConnection::connect(const std::string &host_port,
                                const in_port_t default_port,
                                const bool tcp_no_delay)
    {
        /*
         * Do not allow double connections.
         */
        FswAbortIf(is_connected(), false);
        /*
         * Set up connection information.
         */
        Handle<ConnectionInfo> conn_info = ConnectionInfo::create(
            host_port, default_port, tcp_proto, tcp_no_delay,
            socket_receive_buf_size, socket_send_buf_size);
        FswAbortIfNot(conn_info, false);
        /*
         * Create the Reconnector the first time this method is called.
         */
        if (!reconnector)
        {
            FswAbortIfNot(reconnector.assume_ownership(new Reconnector(fd_bag)),
                          false);
            FswAbortIfNot(reconnector, false);
            FswAbortIfNot(reconnector->connect_sig.connect(
                              make_slot(*this, &TcpConnection::handle_connect)),
                          false);
            FswAbortIfNot(reconnector->timer().set_wait_after_reset(false),
                          false);
            FswAbortIfNot(
                install_dispatch(
                    elist, make_slot(*reconnector, &Reconnector::dispatch)),
                false);
        }
        /*
         * Connect and start the Reconnector.
         */
        FswAbortIfNot(reconnector->connect(conn_info), false);
        FswAbortIfNot(reconnector->timer().reset(), false);
        is_closed_flag = false;
        return true;
    }
    /**
     * Overwrite the ExponentialBackoff instance used for reconnection.
     *
     * @param _timer The ExponentialBackoff to use for reconnection.
     *
     * @return True on success.
     */
    bool TcpConnection::set_reconnect_timer(const ExponentialBackoff &_timer)
    {
        FswAbortIfNot(reconnector, false);
        reconnector->set_timer(_timer);
        return true;
    }
    /**
     * Change the size of the kernel's internal receive buffer for this
     * socket using setsockopt and SO_RCVBUF. This change is made during
     * connect(), so this function must be called before connect().
     *
     * @param _socket_receive_buffer_size The requested receive buffer size.
     *                  Note that the amount of payload data that can be
     *                  buffered in the kernel varies and will be less than
     *                  this size. If <= 0, connect() will not attempt to set
     *                  the receive buffer size and just use the default.
     *
     * @return True on success.
     */
    bool TcpConnection::set_socket_kernel_receive_size(
        int _socket_receive_buffer_size)
    {
        /*
         * The receive buffer size must be set before connecting; see comments
         * above set_socket_receive_buf_size().
         */
        FswAbortIf(is_connected(), false);
        /*
         * Remember for next connect.
         */
        socket_receive_buf_size = _socket_receive_buffer_size;
        return true;
    }
    /**
     * Change the size of the kernel's internal send buffer for this
     * socket using setsockopt and SO_SNDBUF. This change is made during
     * connect(), so this function must be called before connect().
     *
     * @param _socket_send_buffer_size The requested send buffer size.
     *                  Note that the amount of payload data that can be
     *                  buffered in the kernel varies and will be less than
     *                  this size. If <= 0, connect() will not attempt to set
     *                  the send buffer size and just use the default.
     *
     * @return True on success.
     */
    bool
    TcpConnection::set_socket_kernel_send_size(int _socket_send_buffer_size)
    {
        /*
         * It is theoretically possible to allow this function to be called
         * after connect(), but for simplicity and symmetry with
         * set_socket_kernel_receive_size(), only calls before connect() are
         * accepted.
         */
        FswAbortIf(is_connected(), false);
        /*
         * Remember for next connect.
         */
        socket_send_buf_size = _socket_send_buffer_size;
        return true;
    }
    /*
     * See StreamConnection.
     */
    bool TcpConnection::is_connected() const { return !fd_channel.is_closed(); }
    /*
     * See StreamConnection.
     */
    bool TcpConnection::disconnect()
    {
        if (reconnector)
        {
            FswAbortIfNot(reconnector->enable(false), false);
        }
        FswAbortIfNot(fd_channel.close(), false);
        return true;
    }
    /*
     * See StreamConnection.
     */
    bool TcpConnection::reset()
    {
        FswAbortIfNot(fd_channel.close(), false);
        return true;
    }
    /*
     * See StreamConnection.
     */
    bool TcpConnection::is_enabled() const
    {
        if (!reconnector)
            return false;
        return reconnector->is_enabled();
    }
    /*
     * See Channel.
     */
    bool TcpConnection::is_closed() const { return is_closed_flag; }
    /*
     * See Channel.
     */
    bool TcpConnection::is_drained() const
    {
        return is_closed_flag && fd_channel.is_drained();
    }
    /*
     * See Channel.
     */
    bool TcpConnection::is_empty() const { return fd_channel.is_empty(); }
    /*
     * See StreamChannel.
     */
    size_t TcpConnection::get_max_data_len() const
    {
        return fd_channel.get_max_data_len();
    }
    /*
     * See StreamChannel.
     */
    B2c TcpConnection::get_data() const { return fd_channel.get_data(); }
    /*
     * See Channel.
     */
    size_t TcpConnection::space_left() const { return fd_channel.space_left(); }
    /*
     * See StreamConnection.
     */
    size_t TcpConnection::space_taken() const
    {
        return fd_channel.space_taken();
    }
    /*
     * See Channel.
     */
    DataFrame &TcpConnection::get_dataframe(const size_t request_len)
    {
        return fd_channel.get_dataframe(request_len);
    }
    /**
     * Enable the TCP keepalive sub-protocol.
     *
     * The default values are selected to emulate the default behavior of
     * SimpleWatchdog.
     *
     * @param probe_interval The number of seconds a connection must be idle
     *                       before a keepalive probe is sent. Successful
     *                       probes will be initiated no more frequently
     *                       than this number.
     * @param retry_interval The number of seconds before a probe which has
     *                       not been acknowledged is re-transmitted.
     *                       Successful probes will be initiated no more
     *                       frequently than this.
     * @param retry_count Number of consecutive probe responses missed before
     *                    the TCP connection is closed.
     *
     * @return True on success.
     */
    bool TcpConnection::enable_tcp_keepalive(int probe_interval,
                                             int retry_interval,
                                             int retry_count)
    {
        use_keepalive = true;
        keepalive_probe_interval = probe_interval;
        keepalive_retry_interval = retry_interval;
        keepalive_retry_count = retry_count;
        if (is_connected())
        {
            Handle<FdEventSink> fes;
            FswAbortIfNot(fd_channel.get_fd(fes), false);
            FswAbortIfNot(fes, false);
            FswAbortIfNot(Drone::enable_tcp_keepalive(
                              fes->get_fd(), keepalive_probe_interval,
                              keepalive_retry_interval, keepalive_retry_count),
                          false);
        }
        return true;
    }
    /**
     * Disable the TCP keepalive sub-protocol.
     *
     * @return True on success.
     */
    bool TcpConnection::disable_tcp_keepalive()
    {
        use_keepalive = false;
        if (is_connected())
        {
            Handle<FdEventSink> fes;
            FswAbortIfNot(fd_channel.get_fd(fes), false);
            FswAbortIfNot(fes, false);
            FswAbortIfNot(Drone::disable_tcp_keepalive(fes->get_fd()), false);
        }
        return true;
    }
    /**
     * Seal the channel such that no more data can be written.
     *
     * Stops any active reconnection attempts.
     *
     * Note that no written data is lost in this operation; it will
     * continue to be delivered to the client until it is all consumed.
     * Only then will the close signal be emitted.
     *
     * @return True on success.
     */
    bool TcpConnection::channel_close()
    {
        is_closed_flag = true;
        FswAbortIfNot(disconnect(), false);
        return true;
    }
    /*
     * See StreamChannel.
     */
    bool TcpConnection::channel_clear()
    {
        FswAbortIfNot(fd_channel.clear(), false);
        return true;
    }
    /*
     * See Channel.
     */
    bool TcpConnection::channel_commit_dataframe(DataFrame &frame)
    {
        FswAbortIfNot(fd_channel.commit_dataframe(frame), false);
        return true;
    }
    /*
     * See StreamChannel.
     */
    bool TcpConnection::channel_pop_front(const size_t bytes)
    {
        FswAbortIfNot(fd_channel.pop_front(bytes), false);
        has_popped = false;
        return true;
    }
    /**
     * Handle a new connection to a TCP server.
     *
     * Before accepting the new connection, asks clients to read as much
     * data as they can from the old buffer and then clears it. Leaving
     * old stream data in with a new connection would probably result in
     * a corrupt data stream and an unrecoverable error.
     *
     * @param conn_info Contains information about the new connection.
     *
     * @return True on success.
     */
    bool TcpConnection::handle_connect(Handle<ConnectionInfo> conn_info)
    {
        FswAbortIfNot(reconnector, false);
        FswAbortIfNot(reconnector->conn_info, false);
        if (!reconnector->conn_info->is_valid())
            return true;
        FswAbortIfNot(reconnector->conn_info->fd, false);
        FswAbortIf(is_connected(), false);
        /*
         * Give clients one last chance at any remaining data in the channel.
         */
        while (!fd_channel.is_empty())
        {
            has_popped = false;
            if (!signal_read())
                break;
            /*
             * If they stop consuming, we stop.
             */
            if (!has_popped)
                break;
        }
        /*
         * Empty out the channel to make way for the new connection.
         */
        FswAbortIfNot(fd_channel.clear(), false);
        FswAbortIfNot(fd_channel.is_drained(), false);
        /*
         * Assign the fd to our fd_channel and set up event handlers.
         */
        bool success = true;
        FswIfNot2(fd_channel.assign_fd(conn_info->fd), success);
        FswAssert(fd_channel.read_sig.connect(
            make_slot(*this, &TcpConnection::handle_read)));
        FswAssert(fd_channel.write_sig.connect(
            make_slot(*this, &TcpConnection::handle_write)));
        /*
         * Enable/Disable TCP keep alive.
         */
        if (use_keepalive)
        {
            FswIfNot2(Drone::enable_tcp_keepalive(
                          conn_info->fd->get_fd(), keepalive_probe_interval,
                          keepalive_retry_interval, keepalive_retry_count),
                      success);
        }
        else
        {
            FswIfNot2(Drone::disable_tcp_keepalive(conn_info->fd->get_fd()),
                      success);
        }
        /*
         * Keep a watch on the file descriptor so we can emit disconnect
         * signals.
         */
        FswIfNot2(
            conn_info->fd->add_events(
                fd_close_ev, make_slot(*this, &TcpConnection::handle_fd_close)),
            success);
        FswIfNot2(signal_connect(), success);
        /*
         * Clean up if something goes wrong.
         */
        if (!success)
        {
            /*
             * Check that the conn_info's FD still exists - it may have been
             * closed out from under us by someone who handled
             * signal_connect().
             */
            if (conn_info->fd)
                conn_info->fd->close();
            return false;
        }
        return true;
    }
    /**
     * Handle the file descriptor closing.
     *
     * @param fes The FdEventSink.
     * @param fev The FdEvent.
     *
     * @return True on success.
     */
    bool TcpConnection::handle_fd_close(FdEventSink &fes, FdEvent &fev)
    {
        /**
         * The fd_channel's handle_fd_close function should have run
         * before us.
         */
        FswAssert(fd_channel.is_closed());
        /*
         * Notify clients.
         */
        if (FswIfNot(signal_connect()))
        {
            FswAbortIfNot(close(), false);
        }
        else if (is_drained())
        {
            /*
             * If the channel is closed, the FD is closed, and there isn't any
             * data to be read, then emit the close signal. Otherwise, the
             * signal will be emitted after all the data is read.
             */
            FswIfNot(signal_close());
        }
        return true;
    }
    /**
     * Pass read events from the fd_channel up to clients.
     *
     * @param channel The fd_channel.
     *
     * @return True on success.
     */
    bool TcpConnection::handle_read(StreamChannel &channel)
    {
        FswAbortIfNot(&channel == &fd_channel, false);
        FswAbortIfNot(signal_read(), false);
        return true;
    }
    /**
     * Pass write events from the fd_channel up to clients.
     *
     * @param channel The fd_channel.
     *
     * @return True on success.
     */
    bool TcpConnection::handle_write(Channel &channel)
    {
        FswAbortIfNot(&channel == &fd_channel, false);
        FswAbortIfNot(signal_write(), false);
        return true;
    }
} /* end namespace Drone */