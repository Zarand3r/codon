/**
 * @author Stefan Moluf
 * @date   06/15/10
 */
#ifndef SERVER_H
#define SERVER_H
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/Periodic.h"
#include "src/bullwinkle/all/ServerFd.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/net.h"
#include "src/bullwinkle/all/runtime.h"
#include <deque>
namespace Drone
{
    /**
     * A framework for writing connection-oriented servers.
     *
     * The Server class manages setting up the listening socket, accepting
     * new connections, and cleaning up after closed connections. The user
     * simply implements the function which creates the connection-management
     * object (a subclass of ServerFd).
     *
     * The Server can also limit the number of available simultaneous
     * connections. It is important to note that new connections take priority
     * over old connections, so existing clients will be kicked if new
     * clients cause the connection limit to be exceeded. This helps prevent
     * stale connections from locking out legitimate ones.
     *
     * The Server also has a concept of pending and accepted connections. If a
     * ServerFd object is not marked as accepted, then it will be placed on the
     * pending connections list. Pending connections cannot push out accepted
     * connections, but new pending connections can push out old pending
     * connections. Once a connection requests to be accepted, it will be moved
     * from the pending to accepted list.
     */
    class Server : public SignalHandler
    {
    public:
        Server(FdBag &_fdbag, uint _max_connections = UINT_MAX);
        virtual ~Server();
        bool start(in_port_t _port);
        bool start(in_port_t _port, int socket_receive_buffer_size,
                   int socket_send_buffer_size);
        bool start(sockaddr *addr, socklen_t addr_len);
        bool start(sockaddr *addr, socklen_t addr_len,
                   int socket_receive_buffer_size, int socket_send_buffer_size);
        bool start(Handle<FdEventSink> fes);
        bool stop();
        bool is_closed() const;
        in_port_t get_port() const;
        bool set_max_connections(uint _max_connections);
        uint get_max_connections() const;
        uint num_connections() const;
        bool add_connection(Handle<FdEventSink> fes);
        virtual nano_t dispatch(nano_t control_time);

    protected:
        template <class T>
        bool get_connection(uint index, Handle<T> &connection);
        virtual bool server_start();
        virtual bool server_stop();
        virtual nano_t server_dispatch(nano_t control_time);
        virtual bool server_create_connection(Handle<FdEventSink> fes,
                                              Handle<ServerFd> &connection) = 0;
        virtual bool server_finalize_connection(uint index);
        virtual bool server_handle_accepted_connection(uint index);
        virtual bool server_all_disconnect();
        /**
         * Uses this FdBag to register file descriptors.
         */
        FdBag &fdbag;

    private:
        bool server_handle_connect(FdEventSink &fes, FdEvent &fev);
        bool server_handle_disconnect(ServerFd &connection);
        bool server_handle_close(FdEventSink &fes, FdEvent &fev);
        bool server_accept_connection(Handle<ServerFd> connection);
        bool server_close_connection(uint index);
        bool server_close_pending_connection(uint index);
        bool server_prune_connections();
        bool server_prune_pending_connections();
        bool server_prune_accepted_connections();
        /**
         * The Server listens to new connections on this file descriptor.
         */
        Handle<FdEventSink> listen_fd;
        /**
         * The port this Server is currently listening on.
         */
        in_port_t port;
        /**
         * List of accepted connections to this server.
         */
        std::deque<Handle<ServerFd>> connections;
        /**
         * List of pending connections to this server.
         */
        std::deque<Handle<ServerFd>> pending_connections;
        /**
         * Maximum number of simultaneous connections allowed.
         */
        uint max_connections;
        /**
         * Periodic for enforcing a maximum delay in pruning connections.
         */
        Periodic prune_cycle;
        /**
         * True if a connections pruning should be run next cycle.
         */
        bool do_prune_connections;
    };
    /**
     * Retrieve a specific accepted connection by index.
     *
     * @note This is a templated function, so it will accept a Handle of any
     * type. Inheriting classes should pass in a Handle of their own
     * ServerFd type (e.g. Handle<MyServerFd>) and let this method do the
     * casting assignment for them.
     *
     * @note Pending connections cannot be retrieved using this method.
     *
     * @param index Index of connection. Must be less than #num_connections().
     * @param[out] connection Returns connection object.
     *
     * @return True on success.
     */
    template <class T>
    bool Server::get_connection(uint index, Handle<T> &connection)
    {
        FswAbortIf(connection, false);
        FswAbortOutsideRangeUint(index, 0, connections.size(), false);
        /*
         * First we retrieve the connection at the specified index as
         * a ServerFd.
         */
        Handle<ServerFd> temp = connections[index];
        FswAbortIfNot(temp, false);
        /*
         * Then we do a casting assignment and check the result to make
         * sure a cast from ServerFd to T was possible.
         */
        if (!connection.assign_casted(temp))
        {
            FswPrefix();
            dbstring(": Dynamic cast from ServerFd to subclass failed!\n");
            return false;
        }
        return true;
    }
} /* end namespace Drone */
#endif /* SERVER_H */