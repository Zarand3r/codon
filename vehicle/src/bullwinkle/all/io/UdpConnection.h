/**
 * @author Stefan Moluf
 * @date   04/21/11
 */
#ifndef UDP_CONNECTION_H
#define UDP_CONNECTION_H
#include "src/bullwinkle/all/EventList.h"
#include "src/bullwinkle/all/ExponentialBackoffTimer.h"
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/io/DgramConnection.h"
#include "src/bullwinkle/all/io/FdDgramChannel.h"
#include "src/bullwinkle/all/net.h"
#include "src/bullwinkle/all/net_interface_utils.h"
#include "src/bullwinkle/all/net_multicast.h"
namespace Drone
{
    /**
     * A DgramConnection which sends all datagrams to a UDP port. Once
     * connect() has been called, UdpConnection will automatically open a
     * new socket if it goes down for any reason.
     *
     * This class supports IPv4 and IPv6, with the current caveats:
     * - Multicast support for IPv6 has not been added yet and will Sac if
     *   initialized so.
     */
    class UdpConnection : public DgramConnection
    {
    public:
        UdpConnection(EventList &_elist, FdBag &_fd_bag);
        UdpConnection(EventList &_elist, FdBag &_fd_bag,
                      const size_t min_num_dgrams, const size_t max_dgram_len,
                      bool _is_ipv6 = false);
        virtual ~UdpConnection();
        /*
         * Open new connections.
         */
        bool open(const in_port_t my_port = 0, bool _reuse_addr = false,
                  const std::string &my_addr = "");
        bool set_remote_addr(const std::string &host_port,
                             const in_port_t default_port = 0);
        bool connect(const std::string &host_port,
                     const in_port_t default_port = 0,
                     const in_port_t my_port = 0,
                     bool _connected_socket = false, bool _reuse_addr = false,
                     const std::string &my_addr = "");
        bool set_reconnect_timer(const ExponentialBackoff &_timer);
        bool multicast_write_bind(const net_interface_t &interface, bool loop,
                                  UINT8 ttl);
        bool multicast_group_join(const std::string &multicast_address,
                                  const net_interface_t &interface);
        bool
        multicast_group_join(const std::string &multicast_address,
                             const std::vector<net_interface_t> &interfaces);
        bool multicast_group_leave(const std::string &multicast_address,
                                   const net_interface_t &interface);
        bool
        multicast_group_leave(const std::string &multicast_address,
                              const std::vector<net_interface_t> &interfaces);
        bool multicast_recv_all(bool all);
        bool bind_interface(const std::string &interface);
        bool bind_interface(const net_interface_t &interface);
        bool set_broadcast(bool broadcast);
        /*
         * Get status.
         */
        virtual bool is_connected() const;
        virtual bool disconnect();
        bool ipv6() const;
        in_port_t get_port() const;
        bool get_address(sockaddr *const addr, socklen_t *const addr_len) const;
        bool get_address(sockaddr_in *const addr,
                         socklen_t *const addr_len) const;
        bool get_multicast_write_interface(net_interface_t &interface) const;
        bool set_socket_kernel_receive_size(int _socket_receive_buffer_size);
        bool set_socket_kernel_send_size(int _socket_send_buffer_size);
        void set_keep_only_last_dgram(const bool keep_only_last_dgram);
        void set_send_flags(const int send_flags);
        /**
         * @return The the firewall signal from the underlying channel.
         * Connecting to this signal will allow callers to drop dgrams based on
         * the source socket address.
         */
        FdDgramChannel::firewall_signal_t &get_firewall_signal()
        {
            return fd_channel.firewall_signal;
        }
        /**
         * @return The the write error signal from the underlying channel.
         *
         * Connecting to this signal will allow callers to choose to
         * ignore write errors or not.
         */
        FdDgramChannel::write_error_signal_t &get_write_error_signal()
        {
            return fd_channel.write_error_signal;
        }
        /*
         * Non-Channel functions. These enable the use of unconnected sockets
         * over a familiar interface, but can't be directly plumbed into
         * other Channel-oriented classes.
         */
        virtual bool commit_dataframe_to(DataFrame &frame,
                                         const sockaddr_in *const addr,
                                         const socklen_t addr_len);
        virtual bool write_to(const char *data, const size_t data_len,
                              const sockaddr *const addr,
                              const socklen_t addr_len);
        virtual bool write_to(const B2c &data, const sockaddr *const addr,
                              const socklen_t addr_len);
        virtual bool write_to(const char *data, const size_t data_len,
                              const sockaddr_in *const addr,
                              const socklen_t addr_len);
        virtual bool write_to(const B2c &data, const sockaddr_in *const addr,
                              const socklen_t addr_len);
        /*
         * See DgramConnection for complete API.
         */
        virtual bool is_closed() const;
        virtual bool is_drained() const;
        virtual bool is_empty() const;
        virtual B2c peek_dgram() const;
        virtual uint dgrams_avail() const;
        virtual size_t space_left() const;
        virtual DataFrame &get_dataframe(const size_t request_len);

    protected:
        virtual bool channel_close();
        virtual bool channel_clear();
        virtual bool channel_commit_dataframe(DataFrame &frame);
        virtual bool channel_pop_dgram();
        bool create_timer();
        bool open_socket();
        bool connect_socket();
        bool handle_timer(ExponentialBackoffTimer &_timer);
        bool handle_fd_close(FdEventSink &fes, FdEvent &fev);
        bool handle_read(DgramChannel &channel);
        bool handle_write(Channel &channel);
        /**
         * Stores the information needed to configure a file descriptor for a
         * multicast subscription.
         */
        struct multicast_subscription
        {
            multicast_subscription() : group(), interface() {}
            /**
             * The group to join.
             */
            struct sockaddr_in group;
            /**
             * The network interface to join on.
             */
            net_interface_t interface;
        };
        /**
         * Stores the information needed to configure a file descriptor to
         * write multicast data out a particular network interface.
         */
        struct multicast_write_info
        {
            multicast_write_info()
                : interface(), loop(multicast_default_loop),
                  ttl(multicast_default_ttl), is_init(false)
            {}
            /**
             * The network interface to send multicast data with.
             */
            net_interface_t interface;
            /**
             * Whether written multicast data should be looped back to
             * ourselves.
             */
            bool loop;
            /**
             * The desired time-to-live (TTL) for the emitted UDP frames.
             */
            UINT8 ttl;
            /**
             * Whether this struct has been initialized.
             */
            bool is_init;
        };
        /**
         * Attaches an ExponentialBackoffTimer to this EventList.
         */
        EventList &elist;
        /**
         * The FdBag from which to obtain the FDs for this connection.
         */
        FdBag &fd_bag;
        /**
         * Timer which emits connection attempt signals.
         */
        Handle<ExponentialBackoffTimer> timer;
        /**
         * Channel which actually talks to the socket.
         */
        FdDgramChannel fd_channel;
        /**
         * The UDP port that the user requested.
         * A value of 0 means "use ephemeral port."
         */
        in_port_t requested_port;
        /**
         * Address of local host.
         */
        sockaddr_storage local_addr;
        /**
         * Address of remote host.
         */
        sockaddr_storage remote_addr;
        /**
         * Whether this connection is IPv6 or IPv4.
         */
        bool is_ipv6;
        /**
         * Whether this is a connected or unconnected UDP socket.
         */
        bool connected_socket;
        /**
         * Whether reuse of addresses is allowed (i.e., if the SO_REUSEADDR
         * sockopt should be set on the socket before binding).
         */
        bool reuse_addr;
        /**
         * True if the channel is closed and no reconnections are active.
         */
        bool is_closed_flag;
        /**
         * True if disconnect() has been called and we should not try to
         * reconnect.
         */
        bool is_disconnected_flag;
        /**
         * If > 0, set the socket's kernel receive buffer size to this during
         * open().
         */
        int socket_receive_buf_size;
        /**
         * If > 0, set the socket's kernel send buffer size to this during
         * open().
         */
        int socket_send_buf_size;
        /**
         * Collection of multicast groups that this channel listens to. These
         * will be re-asserted whenever fd_channel re-opens its file
         * descriptor.
         */
        std::vector<multicast_subscription> multicast_subscriptions;
        /**
         * The network interface and options that fd_channel's file
         * descriptor should be configured with. Set with
         * multicast_write_bind().
         */
        multicast_write_info multicast_bind;
        /**
         * True if multicast sockets should receive all traffic that the
         * host receives (default socket behavior) for that port, or false
         * to filter traffic to only groups/interfaces subscribed by this
         * socket.
         */
        bool multicast_all;
    };
} /* end namespace Drone */
#endif /* UDP_CONNECTION_H */