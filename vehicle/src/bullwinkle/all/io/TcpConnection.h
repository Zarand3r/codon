/**
 * @author Stefan Moluf
 * @date   04/18/11
 */
#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/Reconnector.h"
#include "src/bullwinkle/all/io/FdStreamChannel.h"
#include "src/bullwinkle/all/io/StreamConnection.h"
namespace Drone
{
    /**
     * A StreamConnection which connects to a TCP port. Once connect()
     * has been called, TcpConnection will do its best to keep the
     * connection alive.
     *
     * WARNING: This class will accept addresses that require a DNS lookup.
     * If reconnection is needed after init, and the DNS lookup times out,
     * it will block for many seconds under default configurations. Node
     * directory entries shortcut DNS lookups, so do not have this problem.
     */
    class TcpConnection : public StreamConnection
    {
    public:
        TcpConnection(EventList &_elist, FdBag &_fd_bag,
                      const size_t size = 1500);
        virtual ~TcpConnection();
        virtual bool connect(const std::string &host_port,
                             const in_port_t default_port = 0,
                             const bool tcp_no_delay = true);
        bool set_reconnect_timer(const ExponentialBackoff &_timer);
        bool set_socket_kernel_receive_size(int _socket_receive_buffer_size);
        bool set_socket_kernel_send_size(int _socket_send_buffer_size);
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
        virtual DataFrame &get_dataframe(const size_t request_len);
        bool enable_tcp_keepalive(int probe_interval = 1,
                                  int retry_interval = 1, int retry_count = 10);
        bool disable_tcp_keepalive();

    protected:
        virtual bool channel_close();
        virtual bool channel_clear();
        virtual bool channel_commit_dataframe(DataFrame &frame);
        virtual bool channel_pop_front(const size_t bytes);

    private:
        bool handle_connect(Handle<ConnectionInfo> conn_info);
        bool handle_fd_close(FdEventSink &fes, FdEvent &fev);
        bool handle_read(StreamChannel &channel);
        bool handle_write(Channel &channel);
        /**
         * Attaches a Reconnector to this EventList.
         */
        EventList &elist;
        /**
         * The FdBag from which to obtain the FDs for this connection.
         */
        FdBag &fd_bag;
        /**
         * Keeps the connection alive.
         */
        Handle<Reconnector> reconnector;
        /**
         * Channel which actually talks to the socket.
         */
        FdStreamChannel fd_channel;
        /**
         * True if the channel is closed and no reconnections are active.
         */
        bool is_closed_flag;
        /**
         * A flag set to true when channel_pop_front is called. Used by
         * handle_connect to detect when data is consumed.
         */
        bool has_popped;
        /**
         * If true then the TCP keepalive protocol will be used.
         */
        bool use_keepalive;
        /**
         * The number of seconds a connection must be idle before a keepalive
         * probe is sent. Successful probes will be initiated no more frequently
         * than this number.
         */
        int keepalive_probe_interval;
        /**
         * The number of seconds before a probe which has not been acknowledged
         * is re-transmitted. Successful probes will be initiated no more
         * frequently than this.
         */
        int keepalive_retry_interval;
        /**
         * Number of consecutive probe responses missed before the TCP
         * connection is closed.
         */
        int keepalive_retry_count;
        /**
         * If > 0, set the socket's kernel receive buffer size to this during
         * connect().
         */
        int socket_receive_buf_size;
        /**
         * If > 0, set the socket's kernel send buffer size to this during
         * connect().
         */
        int socket_send_buf_size;
    };
} /* end namespace Drone */
#endif /* TCP_CONNECTION_H */