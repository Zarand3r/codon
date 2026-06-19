/**
 * @author Stefan Moluf
 * @date   06/02/11
 */
#ifndef TCP_MULTI_SERVER_CONNECTION_H
#define TCP_MULTI_SERVER_CONNECTION_H
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/io/Channel.h"
#include "src/bullwinkle/all/io/FdStreamChannel.h"
#include "src/bullwinkle/all/io/TcpServer.h"
#include "src/bullwinkle/all/net.h"
namespace Drone
{
    /**
     * A channel which hosts a multiplexing TCP server.
     *
     * Since joining streams is not possible, no reads are possible from
     * the server. Clients who write data to the server will be
     * disconnected.
     */
    class TcpMultiServerConnection : public Channel
    {
    public:
        TcpMultiServerConnection(const size_t _size = 1500);
        virtual ~TcpMultiServerConnection();
        bool start(EventList &elist, FdBag &fd_bag, const in_port_t port,
                   const bool atomic_writes,
                   const uint max_connections = UINT_MAX);
        in_port_t get_port() const;
        size_t num_connections() const;
        virtual bool is_closed() const;
        virtual bool is_drained() const;
        virtual size_t space_left() const;
        virtual DataFrame &get_dataframe(const size_t request);
        virtual bool commit_dataframe(DataFrame &frame);

    protected:
        virtual bool channel_close();

    private:
        bool handle_connect(TcpServer &_server,
                            Handle<FdStreamChannel> channel);
        bool handle_read(StreamChannel &channel);
        bool handle_write(Channel &channel);
        bool handle_close(Channel &channel);
        /**
         * The buffer size of all new connection channels.
         */
        const size_t size;
        /**
         * The TcpServer which handles all hosting.
         */
        Handle<TcpServer> server;
        /**
         * Fork which multiplexes data out to all active connections.
         */
        Handle<Channel> fork;
    };
} /* end namespace Drone */
#endif /* TCP_MULTI_SERVER_CONNECTION_H */