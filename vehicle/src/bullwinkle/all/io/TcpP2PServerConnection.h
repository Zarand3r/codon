/**
 * @author Stefan Moluf
 * @date   06/02/11
 */
#ifndef TCP_P2P_SERVER_CONNECTION_H
#define TCP_P2P_SERVER_CONNECTION_H
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/io/FdStreamChannel.h"
#include "src/bullwinkle/all/io/StreamConnection.h"
#include "src/bullwinkle/all/io/TcpServer.h"
#include "src/bullwinkle/all/net.h"
namespace Drone
{
    /**
     * A StreamConnection which hosts a "point-to-point" TCP server. A
     * single connection is allowed to the server at a time, through
     * which all traffic is directed.
     *
     * New connections interrupt and replace old connections.
     */
    class TcpP2PServerConnection : public StreamConnection
    {
    public:
        TcpP2PServerConnection(const size_t _size = 1500);
        virtual ~TcpP2PServerConnection();
        bool start(EventList &elist, FdBag &fd_bag, const in_port_t port);
        bool stop();
        bool set_socket_kernel_receive_size(int _socket_receive_buffer_size);
        bool set_socket_kernel_send_size(int _socket_send_buffer_size);
        in_port_t get_port() const;
        virtual bool is_connected() const;
        virtual bool disconnect();
        virtual bool reset();
        virtual bool is_enabled() const;
        virtual bool is_closed() const;
        virtual bool is_drained() const;
        virtual bool is_empty() const;
        virtual size_t get_max_data_len() const;
        virtual B2c get_data() const;
        virtual size_t space_left() const;
        virtual size_t space_taken() const;
        virtual DataFrame &get_dataframe(const size_t request);

    protected:
        virtual bool channel_clear();
        virtual bool channel_commit_dataframe(DataFrame &frame);
        virtual bool channel_pop_front(const size_t bytes);
        virtual bool channel_close();

    private:
        bool handle_connect(TcpServer &_server,
                            Handle<FdStreamChannel> channel);
        bool handle_read(StreamChannel &channel);
        bool handle_write(Channel &channel);
        bool handle_fd_close(FdEventSink &fes, FdEvent &fev);
        /**
         * The buffer size of all new connection channels.
         */
        const size_t size;
        /**
         * True if the channel is closed and the server is inactive.
         */
        bool is_closed_flag;
        /**
         * If > 0, set the socket's kernel receive buffer size to this.
         */
        int socket_receive_buf_size;
        /**
         * If > 0, set the socket's kernel send buffer size to this.
         */
        int socket_send_buf_size;
        /**
         * The TcpServer which handles all hosting.
         */
        Handle<TcpServer> server;
        /**
         * The currently-active TCP connection.
         */
        Handle<FdStreamChannel> connection;
    };
} /* end namespace Drone */
#endif /* TCP_P2P_SERVER_CONNECTION_H */