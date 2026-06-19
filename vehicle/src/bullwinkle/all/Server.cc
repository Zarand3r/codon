/**
 * @author Stefan Moluf
 * @date   06/15/10
 */
/*
 * This class is infrastructure and doesn't need to hotsync.
 */
#define HOTSYNC_EXEMPT
#include "src/bullwinkle/all/Server.h"
namespace Drone
{
    /**
     * Construct a Server.
     *
     * @param _fdbag Registers file descriptors with this FdBag.
     * @param _max_connections Maximum number of simultaneous connections to
     * allow.
     */
    Server::Server(FdBag &_fdbag, uint _max_connections)
        : fdbag(_fdbag), listen_fd(), port(0), connections(),
          pending_connections(), max_connections(_max_connections),
          prune_cycle("ServerPruneCycle", 1 * billion),
          do_prune_connections(false)
    {}
    Server::~Server() { SacIfNot(stop()); }
    /**
     * Start the server on the given port.
     *
     * @param _port Port to listen on.
     *
     * @return True on success.
     */
    bool Server::start(in_port_t _port) { return start(_port, 0, 0); }
    /**
     * Start the server on the given port.
     *
     * @param _port Port to listen on.
     * @param socket_receive_buffer_size If this is > 0, set the kernel
     *                  receive buffer size to this.
     * @param socket_send_buffer_size If this is > 0, set the kernel
     *                  send buffer size to this.
     *
     * @return True on success.
     */
    bool Server::start(in_port_t _port, int socket_receive_buffer_size,
                       int socket_send_buffer_size)
    {
        SacAbortIf(listen_fd, false);
        /*
         * Construct sockaddr.
         */
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(_port);
        SacAbortIfNot(start(to_sockaddr(&addr), sizeof(addr),
                            socket_receive_buffer_size,
                            socket_send_buffer_size),
                      false);
        return true;
    }
    /**
     * Start the server on the given sockaddr.
     *
     * @param addr Sockaddr to use.
     * @param addr_len Length of \a addr.
     *
     * @return True on success.
     */
    bool Server::start(sockaddr *addr, socklen_t addr_len)
    {
        return start(addr, addr_len, 0, 0);
    }
    /**
     * Start the server on the given sockaddr.
     *
     * @param addr Sockaddr to use.
     * @param addr_len Length of \a addr.
     * @param socket_receive_buffer_size If this is > 0, set the kernel
     *                  receive buffer size to this.
     * @param socket_send_buffer_size If this is > 0, set the kernel
     *                  send buffer size to this.
     *
     * @return True on success.
     */
    bool Server::start(sockaddr *addr, socklen_t addr_len,
                       int socket_receive_buffer_size,
                       int socket_send_buffer_size)
    {
        SacAbortIfNot(addr, false);
        AutoFd fd;
        SacAbortIfNot(net_server(addr, addr_len, tcp_proto,
                                 socket_receive_buffer_size,
                                 socket_send_buffer_size, fd),
                      false);
        SacAbortIfNot(start(fdbag.fd(std::move(fd))), false);
        return true;
    }
    /**
     * Start the server using the given file descriptor. The fd must have been
     * bound to an address and had listen() called on it.
     *
     * @param fes File descriptor to use.
     *
     * @return True on success.
     */
    bool Server::start(Handle<FdEventSink> fes)
    {
        SacAbortIfNot(fes, false);
        SacAbortIf(fes->is_closed(), false);
        /*
         * Store off the listening port of the file descriptor.
         */
        sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        SacAbortOnErrno(
            sx_getsockname(fes->get_fd(), (sockaddr *)&addr, &addr_len), false);
        port = ntohs(addr.sin_port);
        /*
         * Connect signal handlers to the server file descriptor.
         */
        SacAbortIfNot(
            fes->add_events(fd_read_ev,
                            make_slot(*this, &Server::server_handle_connect)),
            false);
        SacAbortIfNot(
            fes->add_events(fd_close_ev,
                            make_slot(*this, &Server::server_handle_close)),
            false);
        /*
         * Store file descriptor.
         */
        listen_fd = fes;
        /*
         * Ask subclass for any extra initialization steps.
         */
        if (SacIfNot(server_start()))
        {
            SacAbortIfNot(stop(), false);
            return false;
        }
        return true;
    }
    /**
     * Stop the server and close all active connections.
     *
     * @return False if an error was encountered.
     */
    bool Server::stop()
    {
        /*
         * Ask subclass for any extra terminating steps.
         */
        SacAbortIfNot(server_stop(), false);
        /*
         * Close all pending conections. The connection_closed flag should not
         * be set here since the closing of a pending connection should not
         * cause the server_all_disconnect() method to be called.
         */
        while (!pending_connections.empty())
        {
            SacIfNot(server_close_pending_connection(0));
            pending_connections.pop_front();
        }
        /*
         * A flag to indicate if at least one connection closed. This is used
         * below to determine if all connections have closed.
         */
        bool connection_closed = false;
        /*
         * Close all conections.
         */
        while (!connections.empty())
        {
            connection_closed = true;
            SacIfNot(server_close_connection(0));
            connections.pop_front();
        }
        /*
         * Check if calling this function has caused a connection to be closed.
         * If one has been closed, we know all have now been closed, so inform
         * the sub class of this.
         */
        if (connection_closed)
        {
            SacIfNot(server_all_disconnect());
        }
        if (!listen_fd)
            return true;
        /*
         * Clear signal handlers on the listen_fd.
         */
        SacAbortIfNot(listen_fd->clear_signals(), false);
        listen_fd->close();
        listen_fd.clear();
        port = 0;
        return true;
    }
    /**
     * Returns true if the server port is closed and no new connections
     * may arrive.
     *
     * @return True if the server port is closed and no new connections
     *         may arrive.
     */
    bool Server::is_closed() const
    {
        if (!listen_fd)
            return true;
        return listen_fd->is_closed();
    }
    /**
     * Returns the port that the Server is currently listening on.
     *
     * @return The port that the Server is currently listening on. 0 if
     *         the Server is stopped.
     */
    in_port_t Server::get_port() const { return port; }
    /**
     * Set the maximum number of simultaneous connections allowed.
     *
     * @param _max_connections Number of connections. Must be nonzero.
     *
     * @return True on success.
     */
    bool Server::set_max_connections(uint _max_connections)
    {
        SacAbortIfEqInt(max_connections, 0, false);
        max_connections = _max_connections;
        return true;
    }
    /**
     * Get the maximum number of simultaneous connections allowed.
     *
     * @return Number of allowed connections.
     */
    uint Server::get_max_connections() const { return max_connections; }
    /**
     * Get the number of active connections.
     *
     * @return Active connections.
     */
    uint Server::num_connections() const { return connections.size(); }
    /**
     * Add a new client connection to the Server.
     *
     * @param fes File descriptor to client.
     *
     * @return True on success.
     */
    bool Server::add_connection(Handle<FdEventSink> fes)
    {
        /*
         * Set up temporary variables.
         */
        Handle<ServerFd> connection;
        bool success = true;
        /*
         * Create a ServerFd object.
         */
        SacIfNot2(server_create_connection(fes, connection), success);
        SacIfNot2(connection, success);
        /*
         * Ensure a valid connection.
         */
        if (connection)
        {
            SacIf2(connection->is_closed(), success);
            SacIfNot2(connection->get_fd() == fes, success);
        }
        /*
         * Close new connection file descriptor if connection was not
         * successfully established.
         */
        if (!success)
        {
            SacIfNot(fes->close());
            return true;
        }
        /*
         * Connect close handler. This should be done for both pending and
         * accepted connections. The server_handle_disconnect() method simply
         * sets the do_prune_connections flag, and we want that to happen for
         * both accepted and pending connections.
         */
        connection->close_sig.connect(
            make_slot(*this, &Server::server_handle_disconnect));
        /*
         * Accept the connection if that has been requested. Otherwise, put the
         * connection on the pending list.
         */
        if (connection->is_acceptance_requested())
        {
            SacIfNot(server_accept_connection(connection));
        }
        else
        {
            pending_connections.push_back(connection);
        }
        /*
         * Run the pruning algorithm. This will handle the case that we have
         * exceeded the connection limit.
         */
        SacAbortIfNot(server_prune_connections(), false);
        return true;
    }
    /**
     * Dispatch the Server.
     *
     * @param control_time Current control time.
     *
     * @return Next wakeup time.
     */
    nano_t Server::dispatch(nano_t control_time)
    {
        nano_t next_time = nano_t_max;
        /*
         * Dispatch subclass and all connections.
         */
        next_time = std::min(server_dispatch(control_time), next_time);
        for (uint i = 0; i < connections.size(); i++)
        {
            Handle<ServerFd> connection = connections[i];
            if (!connection)
                continue;
            next_time = std::min(connection->dispatch(control_time), next_time);
        }
        for (uint i = 0; i < pending_connections.size(); i++)
        {
            Handle<ServerFd> pending_connection = pending_connections[i];
            if (!pending_connection)
                continue;
            next_time =
                std::min(pending_connection->dispatch(control_time), next_time);
        }
        /*
         * Do our actions.
         */
        /*
         * Get the next time from the prune Periodic. This enforces a worst
         * case call-back time for us to prune connections.
         */
        nano_t prune_cycle_next = nano_t_max;
        prune_cycle.is_due(control_time, prune_cycle_next);
        next_time = std::min(prune_cycle_next, next_time);
        /*
         * Pruning is done every call of dispatch(), regardless of whether the
         * Periodic is due. This is to make sure we always clean up and accept
         * connections whenever we have the opportunity to do so.
         */
        SacAbortIfNot(server_prune_connections(), next_time);
        do_prune_connections = false;
        return next_time;
    }
    /**
     * Default no-op handler for extra initialization time procedures.
     *
     * @return True on success.
     */
    bool Server::server_start() { return true; }
    /**
     * Default no-op handler for extra termination procedures.
     *
     * @return True on success.
     */
    bool Server::server_stop() { return true; }
    /**
     * Default no-op handler for extra dispatch procedures.
     *
     * @param control_time Current control time.
     *
     * @return Next wakeup time.
     */
    nano_t Server::server_dispatch(nano_t control_time) { return nano_t_max; }
    /**
     * Default no-op handler for connection finalization.
     *
     * @param index Index of closing connection. Not valid after this method
     *              returns.
     *
     * @return True on success.
     */
    bool Server::server_finalize_connection(uint index) { return true; }
    /**
     * Default no-op handler for connections that have just been accepted. if
     * the connection requested to be accepted upon creation then this method
     * will be called immediately after creation. Otherwise, this will be
     * called after the connection is moved from the pending list to the
     * accepted list.
     *
     * @param index Index of the accepted connection. Not valid after this
     *              method returns.
     *
     * @return True on success.
     */
    bool Server::server_handle_accepted_connection(uint index) { return true; }
    /**
     * Default no-op handler for the all connections closing. This can be used
     * by a sub class to deal with the special case when the Server has
     * transitioned from having at least one connection to having no
     * connections.
     *
     * @note This method will only be called when all accepted connections
     *       close. Pending connections are not considered.
     *
     * @return True on success.
     */
    bool Server::server_all_disconnect() { return true; }
    /**
     * Handle a new connection.
     *
     * @param fes The calling FdEventSink.
     * @param fev The event being handled.
     *
     * @return True on success.
     */
    bool Server::server_handle_connect(FdEventSink &fes, FdEvent &fev)
    {
        SacAbortIfNot(listen_fd, false);
        SacAbortIfNot(fes.get_fd() == listen_fd->get_fd(), false);
        /*
         * Accept new connection.
         */
        sockaddr addr;
        socklen_t addr_len = sizeof(addr);
        memset(&addr, 0, addr_len);
        AutoFd fd(sx_accept(fes.get_fd(), &addr, &addr_len));
        if (fd.get() == -1)
        {
            /*
             * Print out some identifying information about what happened here
             * to help people figure out which Server this is and who was trying
             * to connect.
             */
            SacPrefix();
            dbnprintf(200,
                      ": sx_accept failed in Server::server_handle_connect:\n");
            dbnprintf(500, "-- errno=%d: %s\n", errno, strerror(errno));
            dbnprintf(200, "-- listen port = %d\n", port);
            dbnprintf(200, "-- max_connections = %u\n", max_connections);
            /*
             * Try to turn address into a string. It may not be legit though
             * since accept() failed, so let the user know as well.
             */
            char address_string[200];
            if (!SacIfNot(address_to_string(&addr, addr_len, address_string,
                                            sizeof(address_string))))
            {
                dbnprintf(300, "-- client address: %s\n", address_string);
                dbnprintf(300,
                          "-- (note: since accept() returned an error, the "
                          "client address"
                          " might not be valid)\n");
            }
            return true;
        }
        SacAbortIfNot(add_connection(fdbag.fd(std::move(fd))), false);
        return true;
    }
    /**
     * Handle a disconnect event from one of the ServerFds.
     *
     * @param connection Calling ServerFd.
     *
     * @return True on success.
     */
    bool Server::server_handle_disconnect(ServerFd &connection)
    {
        /*
         * Unfortunately we can't just delete the connection here, since our
         * call path leads back down into the ServerFd and (probably)
         * to the underlying FdEventSink. If we delete their memory, we will
         * return through a deleted context and very likely run into a nest
         * of memory errors.
         *
         * To avoid this, we ask the EventLoop to call us back as soon as
         * possible, at which point we will clean up any connections which
         * have closed without returning through the thing we just deleted.
         */
        do_prune_connections = true;
        return true;
    }
    /**
     * Handle the server file descriptor closing.
     *
     * @param fes The calling FdEventSink.
     * @param fev The received FdEvent.
     *
     * @return True on success.
     */
    bool Server::server_handle_close(FdEventSink &fes, FdEvent &fev)
    {
        SacAbortIfNot(listen_fd, false);
        SacAbortIfNot(fes.get_fd() == listen_fd->get_fd(), false);
        SacPrefix();
        dbstring(": Server listening port closed unexpectedly!\n");
        SacAbortIfNot(stop(), false);
        return true;
    }
    /**
     * Add a connection to the accepted connections list. This will also call
     * server_handle_accepted_connection() and will close the connection if
     * that method fails.
     *
     * @note This will not remove a connection from the pending list. That must
     *       be done separately.
     *
     * @param connection The connection to accept.
     *
     * @return True on success.
     */
    bool Server::server_accept_connection(Handle<ServerFd> connection)
    {
        SacAbortIfNot(connection, false);
        /*
         * This method should only have been called if acceptance was
         * requested. If that is not true then something is wrong and we should
         * just close the connection.
         */
        if (SacIfNot(connection->is_acceptance_requested()))
        {
            SacAbortIfNot(connection->close(), false);
            return false;
        }
        /*
         * Add the connection to the accepted list.
         */
        connections.push_back(connection);
        /*
         * Mark the connection as accepted now that it is on the accepted list.
         * This should only fail if it has already been called, which should
         * not have happened. However, if someone else has called accept before
         * us we should just continue, but emit a warning.
         */
        SacIfNot(connection->accept_connection());
        /*
         * Inform the subclass of Server that this connection has been
         * accepted.
         */
        SacAssert(connections.size() > 0);
        if (SacIfNot(server_handle_accepted_connection(connections.size() - 1)))
        {
            SacAbortIfNot(connection->close(), false);
            return false;
        }
        return true;
    }
    /**
     * Close a ServerFd object.
     *
     * @param index Index of connection to close.
     *
     * @return True on success.
     */
    bool Server::server_close_connection(uint index)
    {
        SacAbortOutsideRangeUint(index, 0, connections.size(), false);
        Handle<ServerFd> connection = connections[index];
        if (!connection)
            return true;
        /*
         * Finalize this connection.
         */
        SacIfNot(server_finalize_connection(index));
        /*
         * If the subclass deleted our connection behind our back, emit a
         * warning, but return true.
         */
        SacAbortIfNot(connection, true);
        /*
         * If the connection is already closed, we're done.
         */
        if (connection->is_closed())
            return true;
        /*
         * Close connection. Only report failure if the closing signal
         * is non-empty.
         */
        const bool close_success = connection->close();
        if (!connection->close_sig.empty())
            SacAbortIfNot(close_success, false);
        return true;
    }
    /**
     * Close a ServerFd object that is on the pending connections list.
     *
     * @param index Index of the pending connection to close.
     *
     * @return True on success.
     */
    bool Server::server_close_pending_connection(uint index)
    {
        SacAbortOutsideRangeUint(index, 0, pending_connections.size(), false);
        Handle<ServerFd> pending_connection = pending_connections[index];
        if (!pending_connection)
            return true;
        /*
         * NOTE: Unlike with connections, pending connections do not have a
         * "finalize" method called on them before they are closed. This is
         * because pending connections are considered more ephemeral, and there
         * is no point in doing extra work for something that was quickly
         * destroyed.
         */
        /*
         * If the connection is already closed, we're done.
         */
        if (pending_connection->is_closed())
            return true;
        /*
         * Close connection. Only report failure if the closing signal
         * is non-empty.
         */
        const bool close_success = pending_connection->close();
        if (!pending_connection->close_sig.empty())
            SacAbortIfNot(close_success, false);
        return true;
    }
    /**
     * Clean up stale connections on the accepted and pending connections lists
     * and enforce the maximum connections limit. This will also check if any
     * pending connections have been marked accepted and will add them to the
     * accepted connections list and remove them from the pending list.
     *
     * @return True on success.
     */
    bool Server::server_prune_connections()
    {
        SacAbortIfNot(server_prune_pending_connections(), false);
        SacAbortIfNot(server_prune_accepted_connections(), false);
        return true;
    }
    /**
     * Clean up stale connections on the pending connections list and enforce
     * the maximum pending connections limit. This will also check if any
     * pending connections have requested to be accepted and will add them to
     * the accepted connections list and remove them from the pending list.
     *
     * @note This should only be called by server_prune_connections()
     *
     * @return True on success.
     */
    bool Server::server_prune_pending_connections()
    {
        /*
         * Remove any closed or otherwise invalid connections. The iteration is
         * back to front to make erasing elements easier.
         *
         * Note that an int is used for the index instead of a size_t because
         * -1 is a valid value if the size of the vector is 0.
         */
        for (int i = pending_connections.size() - 1; i >= 0; i--)
        {
            Handle<ServerFd> pending_connection = pending_connections[i];
            bool is_valid = !!pending_connection;
            if (is_valid)
                is_valid = !!pending_connection->get_fd();
            if (is_valid)
                is_valid = !pending_connection->get_fd()->is_closed();
            if (!is_valid)
            {
                SacIfNot(server_close_pending_connection(i));
                pending_connections.erase(pending_connections.begin() + i);
            }
        }
        /*
         * Add any connections that have requested acceptance to the accepted
         * connections list. The iteration is front to back to preserve
         * ordering (connections should be accepted in the order they arrived).
         *
         * Note that a size_t is used here because the index will never be
         * negative.
         */
        for (size_t i = 0; i < pending_connections.size(); i++)
        {
            Handle<ServerFd> pending_connection = pending_connections[i];
            if (!pending_connection)
            {
                continue;
            }
            if (pending_connection->is_acceptance_requested())
            {
                SacIfNot(server_accept_connection(pending_connection));
            }
        }
        /*
         * Erase any accepted connections from the pending list. These should
         * already have been added to the accepted list from the previous loop.
         * The iteration is back to front to make erasing elements easier.
         *
         * Note that an int is used for the index instead of a size_t because
         * -1 is a valid value if the size of the vector is 0.
         */
        for (int i = pending_connections.size() - 1; i >= 0; i--)
        {
            Handle<ServerFd> pending_connection = pending_connections[i];
            if (!pending_connection)
            {
                continue;
            }
            if (pending_connection->is_connection_accepted())
            {
                pending_connections.erase(pending_connections.begin() + i);
            }
        }
        /*
         * If we still exceed the maximum number of allowed connections,
         * start dropping the oldest ones.
         */
        while (pending_connections.size() > max_connections)
        {
            SacIfNot(server_close_pending_connection(0));
            pending_connections.pop_front();
        }
        return true;
    }
    /**
     * Clean up stale connections on the accepted connections list and enforce
     * the maximum accepted connections limit.
     *
     * @note This should only be called by server_prune_connections()
     *
     * @return True on success.
     */
    bool Server::server_prune_accepted_connections()
    {
        /*
         * A flag to indicate if at least one connection closed. This is used
         * below to determine if all connections have closed.
         */
        bool connection_closed = false;
        /*
         * Remove any closed or otherwise invalid connections. The iteration is
         * back to front to make erasing elements easier.
         *
         * Note that an int is used for the index instead of a size_t because
         * -1 is a valid value if the size of the vector is 0.
         */
        for (int i = connections.size() - 1; i >= 0; i--)
        {
            Handle<ServerFd> connection = connections[i];
            bool is_valid = !!connection;
            if (is_valid)
                is_valid = !!connection->get_fd();
            if (is_valid)
                is_valid = !connection->get_fd()->is_closed();
            if (!is_valid)
            {
                connection_closed = true;
                SacIfNot(server_close_connection(i));
                connections.erase(connections.begin() + i);
            }
        }
        /*
         * If we still exceed the maximum number of allowed connections,
         * start dropping the oldest ones.
         */
        while (connections.size() > max_connections)
        {
            connection_closed = true;
            SacIfNot(server_close_connection(0));
            connections.pop_front();
        }
        /*
         * Check if this prunning has resulted in all connections being lost
         * (i.e. we had a connection that closed and now we have zero
         * connections). If so, inform the sub class.
         */
        if (connection_closed && connections.size() == 0)
        {
            SacIfNot(server_all_disconnect());
        }
        return true;
    }
} /* end namespace Drone */