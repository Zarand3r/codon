/**
 * @author Stefan Moluf
 * @date   06/02/11
 */
#include "src/bullwinkle/all/io/TcpMultiServerConnection.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/io/DgramChannelFork.h"
#include "src/bullwinkle/all/io/StreamChannelFork.h"
namespace Drone
{
    /**
     * Construct a TcpMultiServerConnection.
     *
     * @param _size Size of internal buffers.
     */
    TcpMultiServerConnection::TcpMultiServerConnection(const size_t _size)
        : size(_size), server(), fork()
    {}
    /**
     * TcpMultiServerConnection destructor.
     */
    TcpMultiServerConnection::~TcpMultiServerConnection()
    {
        if (server)
            FswIfNot(server->stop());
        if (fork)
            FswIfNot(fork->close());
    }
    /**
     * Start the server.
     *
     * At this time, the TcpServer will be added to the EventList as an
     * EventSource.
     *
     * @param elist Add tasks to this EventList.
     * @param fd_bag The container of file descriptors to which to
     *               add the TCP server's file descriptor.
     * @param port The port to host on, or 0 for an ephemeral port.
     * @param atomic_writes If true, each write will be treated as
     * atomic, and any particular fork will either see all of it or none
     * of it.
     * @param max_connections The maximum number of simultaneous
     *                        connections.
     *
     * @return True on success.
     */
    bool TcpMultiServerConnection::start(EventList &elist, FdBag &fd_bag,
                                         const in_port_t port,
                                         const bool atomic_writes,
                                         const uint max_connections)
    {
        FswAbortIf(server, false);
        FswAbortIf(fork, false);
        /*
         * Create the fork.
         */
        if (atomic_writes)
        {
            FswAbortIfNot(
                fork.assume_ownership(new DgramChannelFork(1, size, false)),
                false);
        }
        else
        {
            FswAbortIfNot(
                fork.assume_ownership(new StreamChannelFork(size, false)),
                false);
        }
        FswAssert(fork);
        FswAbortIfNot(fork->write_sig.connect(make_slot(
                          *this, &TcpMultiServerConnection::handle_write)),
                      false);
        FswAbortIfNot(fork->close_sig.connect(make_slot(
                          *this, &TcpMultiServerConnection::handle_close)),
                      false);
        /*
         * Create the server.
         */
        FswAbortIfNot(server.assume_ownership(
                          new TcpServer(fd_bag, size, max_connections)),
                      false);
        FswAbortIfNot(server, false);
        /*
         * Connect signals and submit server to the EventList.
         */
        FswAbortIfNot(server->connect_sig.connect(make_slot(
                          *this, &TcpMultiServerConnection::handle_connect)),
                      false);
        FswAbortIfNot(install_dispatch(elist, server), false);
        FswAbortIfNot(server->start(port), false);
        return true;
    }
    /**
     * Returns the port the server is on.
     *
     * @return The port the server is on. 0 if the server is stopped.
     */
    in_port_t TcpMultiServerConnection::get_port() const
    {
        if (!server)
            return 0;
        return server->get_port();
    }
    /**
     * Return the number of active connections.
     *
     * @return Number of active connections.
     */
    size_t TcpMultiServerConnection::num_connections() const
    {
        return server->num_connections();
    }
    /*
     * See Channel.
     */
    bool TcpMultiServerConnection::is_closed() const
    {
        if (!fork)
            return true;
        return fork->is_closed();
    }
    /*
     * See Channel.
     */
    bool TcpMultiServerConnection::is_drained() const
    {
        if (!fork)
            return true;
        return fork->is_drained();
    }
    /*
     * See Channel.
     */
    size_t TcpMultiServerConnection::space_left() const
    {
        if (!fork)
            return 0;
        return fork->space_left();
    }
    /*
     * See Channel.
     */
    DataFrame &TcpMultiServerConnection::get_dataframe(const size_t request)
    {
        if (!fork)
            return DataFrame::null();
        return fork->get_dataframe(request);
    }
    /*
     * See Channel.
     */
    bool TcpMultiServerConnection::commit_dataframe(DataFrame &frame)
    {
        FswAbortIfNot(fork, false);
        FswAbortIfNot(fork->commit_dataframe(frame), false);
        return true;
    }
    /*
     * See Channel.
     */
    bool TcpMultiServerConnection::channel_close()
    {
        if (server)
            FswAbortIfNot(server->stop(), false);
        if (fork)
            FswAbortIfNot(fork->close(), false);
        return true;
    }
    /**
     * Handle a new connection to the server, swapping it in for any
     * existing connection.
     *
     * @param _server The TcpServer.
     * @param channel The new connection.
     *
     * @return True on success.
     */
    bool
    TcpMultiServerConnection::handle_connect(TcpServer &_server,
                                             Handle<FdStreamChannel> channel)
    {
        FswAbortIfNot(channel, false);
        /*
         * Connect our read handler.
         */
        FswAbortIfNot(channel->read_sig.connect(make_slot(
                          *this, &TcpMultiServerConnection::handle_read)),
                      false);
        /*
         * We speculatively cast to the different fork types, because
         * they are not polymorphic at the Fork level. We only do this on
         * new connections, so it shouldn't be too bad.
         */
        Handle<StreamChannelFork> stream_fork;
        Handle<DgramChannelFork> dgram_fork;
        /*
         * Give the channel to the fork.
         */
        if (stream_fork.assign_casted(fork))
        {
            FswAbortIfNot(stream_fork->fork(channel, false), false);
        }
        else if (dgram_fork.assign_casted(fork))
        {
            FswAbortIfNot(dgram_fork->fork(channel, false), false);
        }
        else
        {
            FswAssert(false);
        }
        FswAbortIfNot(signal_write(), false);
        return true;
    }
    /**
     * Close any connections that try to send us data.
     *
     * @param channel The current connection.
     *
     * @return True on success.
     */
    bool TcpMultiServerConnection::handle_read(StreamChannel &channel)
    {
        FswAbortIfNot(channel.close(), false);
        FswAbortIfNot(channel.clear(), false);
        return true;
    }
    /**
     * Pass write signals up to clients.
     *
     * @param channel The current connection.
     *
     * @return True on success.
     */
    bool TcpMultiServerConnection::handle_write(Channel &channel)
    {
        FswAbortIfNot(signal_write(), false);
        return true;
    }
    /**
     * If the underlying channel closes, close ourselves.
     *
     * @param channel The current connection.
     *
     * @return True on success.
     */
    bool TcpMultiServerConnection::handle_close(Channel &channel)
    {
        FswAbortIfNot(close(), false);
        return true;
    }
} /* end namespace Drone */