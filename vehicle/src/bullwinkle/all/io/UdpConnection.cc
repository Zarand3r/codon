/**
 * @author Stefan Moluf
 * @date   04/21/11
 */
#include "src/bullwinkle/all/io/UdpConnection.h"
#include "src/bullwinkle/all/bin_util.h"
#include "src/bullwinkle/all/net.h"
#include "src/bullwinkle/all/net_multicast.h"
#include "src/bullwinkle/all/sock_util.h"
#include <linux/version.h>
namespace Drone
{
    /**
     * Construct a UdpConnection.
     *
     * @param _elist     Run under this EventList.
     * @param _fd_bag    The FdBag from which to obtain the FDs for
     *                   this connection.
     */
    UdpConnection::UdpConnection(EventList &_elist, FdBag &_fd_bag)
        : elist(_elist), fd_bag(_fd_bag), timer(), fd_channel(2, 1500),
          requested_port(0), is_ipv6(false), connected_socket(false),
          reuse_addr(false), is_closed_flag(false), is_disconnected_flag(true),
          socket_receive_buf_size(0), socket_send_buf_size(0),
          multicast_subscriptions(), multicast_bind(), multicast_all(true)
    {
        memset(&local_addr, 0, sizeof(local_addr));
        memset(&remote_addr, 0, sizeof(remote_addr));
    }
    /**
     * Construct a UdpConnection.
     *
     * @param _elist         Run under this EventList.
     * @param _fd_bag        The FdBag from which to obtain the FDs
     *                       for this connection.
     * @param min_num_dgrams Reserve space for at least this many
     *                       datagrams.
     * @param max_dgram_len  Reserve space for datagrams of at most this
     *                       length.
     */
    UdpConnection::UdpConnection(EventList &_elist, FdBag &_fd_bag,
                                 const size_t min_num_dgrams,
                                 const size_t max_dgram_len, bool _is_ipv6)
        : elist(_elist), fd_bag(_fd_bag), timer(),
          fd_channel(min_num_dgrams, max_dgram_len), requested_port(0),
          is_ipv6(_is_ipv6), connected_socket(false), reuse_addr(false),
          is_closed_flag(false), is_disconnected_flag(true),
          socket_receive_buf_size(0), socket_send_buf_size(0),
          multicast_subscriptions(), multicast_bind(), multicast_all(true)
    {
        memset(&local_addr, 0, sizeof(local_addr));
        memset(&remote_addr, 0, sizeof(remote_addr));
    }
    /**
     * UdpConnection destructor.
     */
    UdpConnection::~UdpConnection() { close(); }
    /**
     * Open a new UDP socket on a local port, and return once it's been opened
     * up fully.
     *
     * Clears the remote address.
     *
     * @param my_port Open a socket on this port. Set to 0 to receive a
     *                random ephemeral port.
     * @param _reuse_addr If true, the SO_REUSEADDR sockopt will be set before
     *                    binding.
     * @param my_addr Receive only packets destined for the given address.
     *                Send packets from the given address. Must be in the IPv4
     *                numbers-and-dots notation or IPv6 colon notation. Defaults
     *                to receive packets to any address bound to one of this
     *                computer's network interfaces.
     *
     * @return True on success.
     */
    bool UdpConnection::open(const in_port_t my_port, bool _reuse_addr,
                             const std::string &my_addr)
    {
        /*
         * Do not allow double connections.
         */
        FswAbortIfNot(fd_channel.is_closed(), false);
        if (timer)
        {
            FswAbortIf(timer->is_enabled(), false);
        }
        requested_port = my_port;
        /*
         * Store local host sockaddr.
         */
        memset(&local_addr, 0, sizeof(local_addr));
        if (is_ipv6)
        {
            sockaddr_in6 *local_addr_ipv6 = to_sockaddr_in6(&local_addr);
            local_addr_ipv6->sin6_family = AF_INET6;
            local_addr_ipv6->sin6_port = htons(requested_port);
            FswAbortIfNeqInt(
                1,
                inet_pton(AF_INET6, (my_addr.empty() ? "::" : my_addr.c_str()),
                          &local_addr_ipv6->sin6_addr),
                false);
        }
        else
        {
            sockaddr_in *local_addr_ipv4 = to_sockaddr_in(&local_addr);
            local_addr_ipv4->sin_family = AF_INET;
            local_addr_ipv4->sin_port = htons(requested_port);
            FswAbortIfNot(
                inet_aton((my_addr.empty() ? "0.0.0.0" : my_addr.c_str()),
                          &local_addr_ipv4->sin_addr),
                false);
        }
        /*
         * Clear remote addr if this is not a connected socket.
         */
        if (!connected_socket)
        {
            memset(&remote_addr, 0, sizeof(remote_addr));
        }
        reuse_addr = _reuse_addr;
        /*
         * Open up the socket.
         */
        FswAbortIfNot(open_socket(), false);
        /*
         * Call create_timer() to set up the timer in case we disconnect.
         */
        FswAbortIfNot(create_timer(), false);
        FswAbortIfNot(timer, false);
        is_closed_flag = false;
        is_disconnected_flag = false;
        if (!signal_connect())
            FswIfNot(close());
        return true;
    }
    /**
     * Set the default remote address. This will be used for the default
     * Channel write functions.
     *
     * @param host_port    Send datagrams to this host:port.
     * @param default_port Use this port if none is specified in \a
     *                     host_port.
     *
     * @return True on success.
     */
    bool UdpConnection::set_remote_addr(const std::string &host_port,
                                        const in_port_t default_port)
    {
        FswAbortIf(is_closed(), false);
        /*
         * Store remote host sockaddr.
         */
        FswAbortIfNot(string_to_address(host_port.c_str(), default_port,
                                        to_sockaddr(&remote_addr),
                                        sizeof(remote_addr), is_ipv6),
                      false);
        /*
         * If this is a connected socket, connect it to the remote side.
         */
        if (connected_socket)
        {
            FswAbortIfNot(connect_socket(), false);
        }
        return true;
    }
    /**
     * Open a new socket and connect to a remote host. If \a _connected_socket
     * is true, ::connect() will be called on the socket, causing the kernel
     * to restrict traffic that can pass through it. If \a _connected_socket
     * is false, this is equivalent to open() + set_remote_addr().
     *
     * @param host_port    Send datagrams to this host:port.
     * @param default_port Use this port if none is specified in \a
     *                     host_port.
     * @param my_port      Open a socket on this port. Set to 0 to
     *                     receive a random ephemeral port.
     * @param _connected_socket Whether to actually call ::connect() on this
     *                     socket. If false, the remote address will be
     *                     stored, as in set_remote_addr().
     * @param _reuse_addr If true, the SO_REUSEADDR sockopt will be set before
     *                    binding.
     * @param my_addr Receive only packets destined for the given address.
     *                Send packets from the given address. Must be in the IPv4
     *                numbers-and-dots notation or IPv6 colon notation. Defaults
     *                to receive packets to any address bound to one of this
     *                computer's network interfaces.
     *
     * @return True on success.
     */
    bool UdpConnection::connect(const std::string &host_port,
                                const in_port_t default_port,
                                const in_port_t my_port,
                                bool _connected_socket /* = false */,
                                bool _reuse_addr /* = false */,
                                const std::string &my_addr)
    {
        /*
         * Do not allow double connections.
         */
        FswAbortIfNot(fd_channel.is_closed(), false);
        connected_socket = _connected_socket;
        /*
         * Open local socket.
         */
        FswAbortIfNot(open(my_port, _reuse_addr, my_addr), false);
        /*
         * Store remote host sockaddr.
         */
        FswAbortIfNot(set_remote_addr(host_port, default_port), false);
        return true;
    }
    /**
     * Overwrite the ExponentialBackoff instance used for reconnection.
     *
     * @param _timer The ExponentialBackoff to use for reconnection.
     *
     * @return True on success.
     */
    bool UdpConnection::set_reconnect_timer(const ExponentialBackoff &_timer)
    {
        FswAbortIfNot(timer, false);
        timer->backoff = _timer;
        return true;
    }
    /**
     * Bind this UdpConnection to a particular network interface for sending
     * multicast data. Note: this applies to all multicast data sent over this
     * UdpConnection.
     *
     * @param interface The network interface to associate the socket's
     *                  multicast data with.
     *
     * @param loop Whether traffic should be looped back to this socket.
     *
     * @param ttl The desired time-to-live (TTL) for the emitted UDP frames.
     *
     * @return True on success.
     */
    bool UdpConnection::multicast_write_bind(const net_interface_t &interface,
                                             bool loop, UINT8 ttl)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        Handle<FdEventSink> fes;
        FswAbortIfNot(fd_channel.get_fd(fes), false);
        if (dbverbose() >= 2)
        {
            dbnprintf(300, "Binding FD %d to interface %s\n", fes->get_fd(),
                      interface.name.c_str());
        }
        multicast_bind.interface = interface;
        multicast_bind.loop = loop;
        multicast_bind.ttl = ttl;
        multicast_bind.is_init = true;
        FswAbortIfNot(multicast_set_interface(fes->get_fd(), interface, loop),
                      false);
        FswAbortIfNot(multicast_set_ttl(fes->get_fd(), ttl), false);
        return true;
    }
    /**
     * Bind this UdpConnection to a particular network interface for receiving
     * from a Multicast group.
     *
     * This group subscription will be saved and re-configured in
     * open_socket(), should the FD ever close.
     *
     * @param multicast_address The IP address of the Multicast group to
     *                          subscribe to.
     *
     * @param interface The network interface to associate the socket with.
     *
     * @return True on success.
     */
    bool
    UdpConnection::multicast_group_join(const std::string &multicast_address,
                                        const net_interface_t &interface)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        /*
         * Store this subscription.
         */
        multicast_subscription subscription;
        subscription.interface = interface;
        FswAbortIfNot(string_to_address(multicast_address.c_str(), 0U,
                                        &subscription.group,
                                        sizeof(subscription.group)),
                      false);
        multicast_subscriptions.push_back(subscription);
        if (is_connected())
        {
            Handle<FdEventSink> fes;
            FswAbortIfNot(fd_channel.get_fd(fes), false);
            FswAbortIfNot(multicast_subscribe(fes->get_fd(), subscription.group,
                                              interface),
                          false);
        }
        return true;
    }
    /**
     * Bind this UdpConnection to a set of network interfaces for receiving
     * from a Multicast group.
     *
     * @param multicast_address The IP address of the Multicast group to
     *                          subscribe to.
     *
     * @param interfaces The network interfaces to associate the socket with.
     *
     * @return True on success.
     */
    bool UdpConnection::multicast_group_join(
        const std::string &multicast_address,
        const std::vector<net_interface_t> &interfaces)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        for (size_t i = 0; i < interfaces.size(); i++)
        {
            FswAbortIfNot(
                multicast_group_join(multicast_address, interfaces[i]), false);
        }
        return true;
    }
    /**
     * Unsubscribe this UdpConnection from a multicast address group.
     *
     * @param multicast_address The IP address of the Multicast group to
     * unsubscribe from.
     *
     * @param interface The network interface to associate the socket with.
     *
     * @return True on success.
     */
    bool
    UdpConnection::multicast_group_leave(const std::string &multicast_address,
                                         const net_interface_t &interface)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        struct sockaddr_in group;
        FswAbortIfNot(string_to_address(multicast_address.c_str(), 0U, &group,
                                        sizeof(group)),
                      false);
        /*
         * Remove this subscription from our internal storage.
         */
        std::vector<multicast_subscription>::iterator iter =
            multicast_subscriptions.begin();
        while (iter != multicast_subscriptions.end())
        {
            const multicast_subscription &sub = *iter;
            if ((sub.group.sin_addr.s_addr == group.sin_addr.s_addr) &&
                (sub.interface == interface))
            {
                iter = multicast_subscriptions.erase(iter);
            }
            else
            {
                iter++;
            }
        }
        if (is_connected())
        {
            Handle<FdEventSink> fes;
            FswAbortIfNot(fd_channel.get_fd(fes), false);
            FswAbortIfNot(multicast_unsubscribe(fes->get_fd(),
                                                multicast_address, interface),
                          false);
        }
        return true;
    }
    /**
     * Unsubscribe this UdpConnection from a multicast address group on
     * multiple interfaces.
     *
     * @param multicast_address The IP address of the Multicast group to
     *                          unsubscribe from.
     *
     * @param interfaces The network interfaces to unsubscribe the socket from
     *                   the group on.
     *
     * @return True on success.
     */
    bool UdpConnection::multicast_group_leave(
        const std::string &multicast_address,
        const std::vector<net_interface_t> &interfaces)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        for (size_t i = 0; i < interfaces.size(); i++)
        {
            FswAbortIfNot(
                multicast_group_leave(multicast_address, interfaces[i]), false);
        }
        return true;
    }
    /**
     * Sets the IP_MULTICAST_ALL option for this socket. When true (default),
     * the socket receives multicast traffic for all multicast groups that
     * the _host_ receives (this is generally the union of all sockets
     * subscriptions). When false, the socket receives multicast traffic for
     * only the groups/interfaces that this socket has explicitly joined (with
     * multicast_group_join()).
     *
     * This setting will persist across open/disconnect of this UdpConnection.
     *
     * @param all The value to set for IP_MULTICAST_ALL.
     *
     * @return True on success.
     */
    bool UdpConnection::multicast_recv_all(bool all)
    {
        FswMsgAbortIf(is_ipv6, false, 200,
                      "IPv6 multicast is currently not supported.");
        /*
         * This #if is required to maintain compatibility with
         * the datarecorder which is running an old kernel version.
         * When building with platbundles, the LINUX_VERSION_CODE
         * contains the correct kernel version of the target.
         */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
#define IP_MULTICAST_ALL_SUPPORTED
#endif
#ifdef IP_MULTICAST_ALL_SUPPORTED
        Handle<FdEventSink> fes;
        FswAbortIfNot(fd_channel.get_fd(fes), false);
        int all_int = static_cast<int>(all);
        FswAbortOnErrno(fsw_setsockopt(fes->get_fd(), IPPROTO_IP,
                                      IP_MULTICAST_ALL, &all_int,
                                      sizeof(all_int)),
                        false);
        /*
         * Store the value to be reapplied if the socket re-opens.
         */
        multicast_all = all;
#endif
        return true;
    }
    /**
     * Binds this socket to the specified interface using SO_BINDTODEVICE. This
     * means that the socket will only receive packets if they arrived via the
     * specified interface.
     *
     * @param interface The name of the interface to bind to.
     *
     * @return True on success.
     */
    bool UdpConnection::bind_interface(const std::string &interface)
    {
        Handle<FdEventSink> fes;
        FswAbortIfNot(fd_channel.get_fd(fes), false);
        FswAbortIfNot(fes, false);
        FswAbortOnErrno(
            fsw_setsockopt(fes->get_fd(), SOL_SOCKET, SO_BINDTODEVICE,
                          (void *)interface.c_str(), interface.size()),
            false);
        return true;
    }
    /**
     * @see bind_interface(const std::string &)
     */
    bool UdpConnection::bind_interface(const net_interface_t &interface)
    {
        FswAbortIfNot(bind_interface(interface.name), false);
        return true;
    }
    /**
     * Enables sending broadcast packets out of this socket. Without calling
     * this method, outgoing broadcast packets will be dropped.
     *
     * @param broadcast True to enable outgoing broadcast packets.
     *
     * @return True on success.
     */
    bool UdpConnection::set_broadcast(bool broadcast)
    {
        Handle<FdEventSink> fes;
        FswAbortIfNot(fd_channel.get_fd(fes), false);
        FswAbortIfNot(fes, false);
        int tmp = static_cast<int>(broadcast);
        FswAbortOnErrno(fsw_setsockopt(fes->get_fd(), SOL_SOCKET, SO_BROADCAST,
                                      &tmp, sizeof(tmp)),
                        false);
        return true;
    }
    /*
     * See DgramConnection.
     */
    bool UdpConnection::is_connected() const { return !fd_channel.is_closed(); }
    /*
     * See DgramConnection.
     */
    bool UdpConnection::disconnect()
    {
        is_disconnected_flag = true;
        if (timer)
        {
            FswAbortIfNot(timer->disable(), false);
        }
        FswAbortIfNot(fd_channel.close(), false);
        return true;
    }
    /**
     * Change the size of the kernel's internal receive buffer for this
     * socket using setsockopt and SO_RCVBUF. This setting is applied during
     * open(), so this function must be called before open(). If the
     * requested buffer size is <= 0, open_socket() will not attempt to
     * set the receive buffer size and just use the default.
     *
     * @param _socket_receive_buffer_size The requested receive buffer size.
     *                  Note that the amount of payload data that can be
     *                  buffered in the kernel varies and will be less than
     *                  this size. If <= 0, open_socket() will not
     *                  attempt to set the receive buffer size and just
     *                  use the default.
     *
     * @return True on success.
     */
    bool UdpConnection::set_socket_kernel_receive_size(
        int _socket_receive_buffer_size)
    {
        FswAbortIf(is_connected(), false);
        socket_receive_buf_size = _socket_receive_buffer_size;
        return true;
    }
    /**
     * Change the size of the kernel's internal send buffer for this
     * socket using setsockopt and SO_RCVBUF. This setting is applied during
     * open(), so this function must be called before open(). If the
     * requested buffer size is <= 0, open_socket() will not attempt to
     * set the send buffer size and just use the default.
     *
     * @param _socket_send_buffer_size
     *                  The requested send buffer size. Note that the amount of
     *                  payload data that can be buffered in the kernel varies
     *                  and will be less than this size. If <= 0, open_socket()
     *                  will not attempt to set the send buffer size and just
     *                  use the default.
     *
     * @return True on success.
     */
    bool
    UdpConnection::set_socket_kernel_send_size(int _socket_send_buffer_size)
    {
        FswAbortIf(is_connected(), false);
        socket_send_buf_size = _socket_send_buffer_size;
        return true;
    }
    /**
     * Controls whether the underlying channel should drop all but the last
     * datagram received after each read.
     *
     * @param keep_only_last_dgram Whether to drop all but the last datagram on
     *                             reads.
     */
    void
    UdpConnection::set_keep_only_last_dgram(const bool keep_only_last_dgram)
    {
        fd_channel.set_keep_only_last_dgram(keep_only_last_dgram);
    }
    /**
     * Set flags arg for the call to sendto().
     *
     * @param send_flags Bitwise OR of flags for sendto() syscall.
     */
    void UdpConnection::set_send_flags(const int send_flags)
    {
        fd_channel.set_send_flags(send_flags);
    }
    /**
     * Returns whether this is an IPv6 connection or not.
     *
     * #return True for IPv6 connection.
     */
    bool UdpConnection::ipv6() const { return is_ipv6; }
    /**
     * Returns the local port of the UDP socket.
     *
     * @return The local port of the UDP socket.
     */
    in_port_t UdpConnection::get_port() const
    {
        FswAbortIf(is_closed(), 0);
        if (is_ipv6)
        {
            return ntohs(to_sockaddr_in6(&local_addr)->sin6_port);
        }
        else
        {
            return ntohs(to_sockaddr_in(&local_addr)->sin_port);
        }
    }
    /**
     * Get the sockaddr of the first datagram in the channel.
     *
     * @param[out]    addr     Writes sockaddr to this pointer.
     * @param[in,out] addr_len Indicates the size of \a addr. Returns the
     *                         length of the copied data.
     *
     * @return True if a datagram is in the channel and \a addr_len is
     *         sufficient to hold the sockaddr for this datagram.
     */
    bool UdpConnection::get_address(sockaddr *const addr,
                                    socklen_t *const addr_len) const
    {
        if (connected_socket)
        {
            if (is_ipv6)
            {
                FswAbortIfNotOpUint(*addr_len, >=, sizeof(sockaddr_in6), false);
                sockaddr_in6 *const addr_in6 = to_sockaddr_in6(addr);
                *addr_in6 = *to_sockaddr_in6(&remote_addr);
                *addr_len = sizeof(sockaddr_in6);
            }
            else
            {
                FswAbortIfNotOpUint(*addr_len, >=, sizeof(sockaddr_in), false);
                sockaddr_in *const addr_in = to_sockaddr_in(addr);
                *addr_in = *to_sockaddr_in(&remote_addr);
                *addr_len = sizeof(sockaddr_in);
            }
        }
        else
        {
            FswAbortIfNot(fd_channel.get_address(addr, addr_len), false);
        }
        return true;
    }
    /**
     * Get the sockaddr_in of the first datagram in the channel. This version of
     * the function only works with IPv4 connections.
     *
     * @param[out]    addr     Writes sockaddr_in to this pointer.
     * @param[in,out] addr_len Indicates the size of \a addr. Returns the
     *                         length of the copied data.
     *
     * @return True if a datagram is in the channel and \a addr_len is
     *         sufficient to hold the sockaddr for this datagram.
     */
    bool UdpConnection::get_address(sockaddr_in *const addr,
                                    socklen_t *const addr_len) const
    {
        FswAbortIf(is_ipv6, false);
        FswAbortIfNot(get_address(to_sockaddr(addr), addr_len), false);
        return true;
    }
    /**
     * Returns the current bound multicast write interface, if it has been set
     *
     * @param[out] interface The interface previously passed to
     * mcast_write_bind.
     *
     * @return True if this connection has a bound multicast write interface,
     * false if not. If this function returns false, the interface argument will
     * not be changed.
     */
    bool UdpConnection::get_multicast_write_interface(
        net_interface_t &interface) const
    {
        FswAbortIfNot(multicast_bind.is_init, false);
        interface = multicast_bind.interface;
        return true;
    }
    /**
     * Finalize a write to the buffer. Must be called after writing into
     * the DataFrame object returned by get_dataframe(), or all written data
     * will be discarded.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param frame    Commit the contents of this object. Must be the
     *                 same object that was returned from
     *                 get_dataframe().
     * @param addr     Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return True on success.
     */
    bool UdpConnection::commit_dataframe_to(DataFrame &frame,
                                            const sockaddr_in *const addr,
                                            const socklen_t addr_len)
    {
        FswAbortIf(is_ipv6, false);
        if (FswIf(connected_socket))
        {
            FswPrefix();
            dbstring(": Cannot use commit_dataframe_to() on a "
                     "connected socket.\n");
            return false;
        }
        FswAbortIfNot(fd_channel.commit_dataframe_to(
                          frame, (const sockaddr *)addr, addr_len),
                      false);
        return true;
    }
    /**
     * Write a datagram into the channel.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data     Pointer to the datagram.
     * @param data_len Length of the datagram.
     * @param addr     Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool UdpConnection::write_to(const char *data, const size_t data_len,
                                 const sockaddr *const addr,
                                 const socklen_t addr_len)
    {
        /*
         * It is an error to try to specify a different host when writing to a
         * connected socket, so write_to() cannot be used.
         */
        if (FswIf(connected_socket))
        {
            FswPrefix();
            dbstring(": Cannot use write_to() on a connected socket.\n");
            return false;
        }
        FswAbortIfNot(fd_channel.write_to(data, data_len, addr, addr_len),
                      false);
        return true;
    }
    /**
     * Write a datagram into the channel.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data     Pointer to the datagram.
     * @param addr     Pointer to the destination sockaddr.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool UdpConnection::write_to(const B2c &data, const sockaddr *const addr,
                                 const socklen_t addr_len)
    {
        /*
         * It is an error to try to specify a different host when writing to a
         * connected socket, so write_to() cannot be used.
         */
        if (FswIf(connected_socket))
        {
            FswPrefix();
            dbstring(": Cannot use write_to() on a connected socket.\n");
            return false;
        }
        FswAbortIfNot(
            fd_channel.write_to(data.buf(), data.len(), addr, addr_len), false);
        return true;
    }
    /**
     * Write a datagram into the channel. This version of the function only
     * works with IPv4 connections.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data     Pointer to the datagram.
     * @param data_len Length of the datagram.
     * @param addr     Pointer to the destination sockaddr_in.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool UdpConnection::write_to(const char *data, const size_t data_len,
                                 const sockaddr_in *const addr,
                                 const socklen_t addr_len)
    {
        FswAbortIf(is_ipv6, false);
        FswAbortIfNot(write_to(data, data_len, to_sockaddr(addr), addr_len),
                      false);
        return true;
    }
    /**
     * Write a datagram into the channel. This version of the function only
     * works with IPv4 connections.
     *
     * Allows the writer to specify a destination sockaddr for this datagram.
     *
     * @param data     Pointer to the datagram.
     * @param addr     Pointer to the destination sockaddr_in.
     * @param addr_len Length of \a addr.
     *
     * @return False if the channel is closed or if there is not enough
     *         space in the channel.
     */
    bool UdpConnection::write_to(const B2c &data, const sockaddr_in *const addr,
                                 const socklen_t addr_len)
    {
        FswAbortIf(is_ipv6, false);
        FswAbortIfNot(write_to(data, to_sockaddr(addr), addr_len), false);
        return true;
    }
    /*
     * See Channel.
     */
    bool UdpConnection::is_closed() const { return is_closed_flag; }
    /*
     * See Channel.
     */
    bool UdpConnection::is_drained() const
    {
        return is_closed_flag && fd_channel.is_drained();
    }
    /*
     * See Channel.
     */
    bool UdpConnection::is_empty() const { return fd_channel.is_empty(); }
    /*
     * See DgramChannel.
     */
    B2c UdpConnection::peek_dgram() const { return fd_channel.peek_dgram(); }
    /*
     * See DgramChannel.
     */
    uint UdpConnection::dgrams_avail() const
    {
        return fd_channel.dgrams_avail();
    }
    /*
     * See Channel.
     */
    size_t UdpConnection::space_left() const { return fd_channel.space_left(); }
    /*
     * See Channel.
     */
    DataFrame &UdpConnection::get_dataframe(const size_t request_len)
    {
        return fd_channel.get_dataframe(request_len);
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
    bool UdpConnection::channel_close()
    {
        is_closed_flag = true;
        FswAbortIfNot(disconnect(), false);
        return true;
    }
    /*
     * See DgramChannel.
     */
    bool UdpConnection::channel_clear()
    {
        FswAbortIfNot(fd_channel.clear(), false);
        return true;
    }
    /**
     * Finalize a write to the buffer. Must be called after writing into
     * the DataFrame object returned by get_dataframe(), or all written data
     * will be discarded.
     *
     * The datagram will be sent to the remote address set with
     * set_remote_addr() or connect().
     *
     * @param write Commit the contents of this object. Must be the same
     *              object that was returned from get_dataframe().
     *
     * @return False if no remote address has been set, or on error.
     */
    bool UdpConnection::channel_commit_dataframe(DataFrame &frame)
    {
        bool no_remote_address = false;
        if (is_ipv6)
        {
            in6_addr empty_in6_addr = {};
            no_remote_address =
                (0 == memcmp(to_sockaddr_in6(&remote_addr)->sin6_addr.s6_addr,
                             &empty_in6_addr, sizeof(in6_addr)));
        }
        else
        {
            no_remote_address =
                (0 == to_sockaddr_in(&remote_addr)->sin_addr.s_addr);
        }
        if (no_remote_address)
        {
            FswPrefix();
            dbstring(": no remote address specified.\n");
            return false;
        }
        if (connected_socket)
        {
            FswAbortIfNot(fd_channel.commit_dataframe(frame), false);
        }
        else
        {
            const socklen_t remote_addr_len =
                (is_ipv6 ? sizeof(sockaddr_in6) : sizeof(sockaddr_in));
            FswAbortIfNot(
                fd_channel.commit_dataframe_to(frame, to_sockaddr(&remote_addr),
                                               remote_addr_len),
                false);
        }
        return true;
    }
    /*
     * See DgramChannel.
     */
    bool UdpConnection::channel_pop_dgram()
    {
        FswAbortIfNot(fd_channel.get_dgram().pop(), false);
        return true;
    }
    /**
     * Connection helper method. Creates (if necessary) the exponential
     * connection timer.
     *
     * @return True on success.
     */
    bool UdpConnection::create_timer()
    {
        /*
         * Create timer the first time this method is called.
         */
        if (!timer)
        {
            FswAbortIfNot(timer.assume_ownership(new ExponentialBackoffTimer(
                              billion, 16 * billion, 3 * billion, false)),
                          false);
            FswAbortIfNot(timer, false);
            FswAbortIfNot(timer->timer_sig.connect(
                              make_slot(*this, &UdpConnection::handle_timer)),
                          false);
            FswAbortIfNot(
                install_dispatch(
                    elist,
                    make_slot(*timer, &ExponentialBackoffTimer::dispatch)),
                false);
        }
        FswAbortIfNot(timer->backoff.reset(), false);
        return true;
    }
    /**
     * Open a new UDP socket with local_addr.
     *
     * Will replace the contents of local_addr with fsw_getsockanme if the
     * socket was successfully created.
     *
     * @return True on success.
     */
    bool UdpConnection::open_socket()
    {
        /**
         * Do not allow double connections.
         */
        FswAbortIf(is_connected(), false);
        /*
         * Create unconnected dgram socket.
         */
        AutoFd fd;
        FswAbortOnErrno(fd = socket(local_addr.ss_family, SOCK_DGRAM, 0),
                        false);
        /*
         * Use non-blocking sockets so that FdDgramChannel will attempt
         * multiple writes per wakeup.
         */
        FswAbortIfNot(fsw_set_nonblock(fd.get()), false);
        const bool is_nonblocking = true;
        /*
         * Set buffer sizes if requested.
         */
        if (socket_receive_buf_size > 0)
        {
            FswAbortIfNot(
                set_socket_receive_buf_size(fd.get(), socket_receive_buf_size),
                false);
        }
        /*
         * Set buffer sizes if requested.
         */
        if (socket_send_buf_size > 0)
        {
            FswAbortIfNot(
                set_socket_send_buf_size(fd.get(), socket_send_buf_size),
                false);
        }
        /*
         * Before binding, optionally set the SO_REUSEADDR socket option.
         */
        if (reuse_addr)
        {
            int state = 1;
            FswAbortOnErrno(fsw_setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR,
                                          (void *)&state, sizeof(state)),
                            false);
        }
        /*
         * Re-set sin_port in the address structure.  This handles the case
         * where the user asked for an ephemeral port (requested_port == 0),
         * but we later filled in the structure with the actual port (see
         * fsw_getsockname() below) so that get_port() works.
         *
         * In short, we do not want to request the same ephemeral port if
         * we need to re-open the socket.  That could fail if another
         * process races and happens to grab the same ephemeral port (see
         * ticket #sw8828).
         */
        if (is_ipv6)
        {
            to_sockaddr_in6(&local_addr)->sin6_port = htons(requested_port);
        }
        else
        {
            to_sockaddr_in(&local_addr)->sin_port = htons(requested_port);
        }
        /*
         * Bind the socket to the port.
         */
        if (bind(fd.get(), to_sockaddr(&local_addr), sizeof(local_addr)) != 0)
        {
            /*
             * Save the errno because if address_to_string fails, it
             * will change it.
             */
            int errno_saved = errno;
            FswPrefix();
            char addr_string[1024];
            if (!address_to_string(&local_addr, sizeof(local_addr), addr_string,
                                   sizeof(addr_string)))
            {
                strncpy(addr_string, "unknown", sizeof(addr_string));
            }
            dbnprintf(100, ": bind(%s) failed: (%d) %s\n", addr_string,
                      errno_saved, strerror(errno_saved));
            return false;
        }
        /*
         * Get the socket name in case we originally asked for an
         * ephemeral port.  This allows us to find out which port
         * we received via the get_port() method.
         */
        socklen_t local_addr_len = sizeof(local_addr);
        FswAbortOnErrno(
            fsw_getsockname(fd.get(), to_sockaddr(&local_addr), &local_addr_len),
            false);
        /*
         * Re-configure any multicast settings.
         */
        if (multicast_bind.is_init)
        {
            FswAbortIfNot(multicast_set_interface(fd.get(),
                                                  multicast_bind.interface,
                                                  multicast_bind.loop),
                          false);
            FswAbortIfNot(multicast_set_ttl(fd.get(), multicast_bind.ttl),
                          false);
        }
        if (!multicast_all)
        {
            /*
             * Only call this if multicast_all==false. Otherwise, keep the
             * default behavior for sockets.
             */
            int all_int = static_cast<int>(multicast_all);
            FswAbortOnErrno(fsw_setsockopt(fd.get(), IPPROTO_IP,
                                          IP_MULTICAST_ALL, &all_int,
                                          sizeof(all_int)),
                            false);
        }
        for (size_t i = 0; i < multicast_subscriptions.size(); i++)
        {
            const multicast_subscription &ms = multicast_subscriptions[i];
            FswAbortIfNot(multicast_subscribe(fd.get(), ms.group, ms.interface),
                          false);
        }
        Handle<FdEventSink> fes = fd_bag.fd(std::move(fd));
        FswAbortIfNot(fes, false);
        /*
         * Hand the socket to the fd_channel.
         */
        bool success = true;
        dgram_fd_type_t socket_type =
            connected_socket ? dgram_fd_conn_socket : dgram_fd_uconn_socket;
        FswIfNot2(fd_channel.assign_fd(fes, socket_type, is_nonblocking),
                  success);
        /*
         * Connect our event handlers.
         */
        FswIfNot2(
            fes->add_events(fd_close_ev,
                            make_slot(*this, &UdpConnection::handle_fd_close)),
            success);
        FswIfNot2(fd_channel.read_sig.connect(
                      make_slot(*this, &UdpConnection::handle_read)),
                  success);
        FswIfNot2(fd_channel.write_sig.connect(
                      make_slot(*this, &UdpConnection::handle_write)),
                  success);
        return success;
    }
    /**
     * Call ::connect() on the socket to connect it to a remote host. This can
     * only be called if the connected_socket flag is true and the remote_addr
     * variable has been filled in (i.e. set_remote_addr() has been called).
     *
     * @return True on success.
     */
    bool UdpConnection::connect_socket()
    {
        /*
         * Make sure this is a connected socket.
         */
        FswAbortIfNot(connected_socket, false);
        /*
         * Make sure the remote address has been set.
         */
        bool no_remote_address = false;
        if (is_ipv6)
        {
            in6_addr empty_in6_addr = {};
            no_remote_address =
                (0 == memcmp(to_sockaddr_in6(&remote_addr)->sin6_addr.s6_addr,
                             &empty_in6_addr, sizeof(in6_addr)));
        }
        else
        {
            no_remote_address =
                (0 == to_sockaddr_in(&remote_addr)->sin_addr.s_addr);
        }
        if (no_remote_address)
        {
            FswPrefix();
            dbstring(": Socket connect failed. "
                     "No remote address specified.\n");
            return false;
        }
        /*
         * Connect the socket.
         */
        Handle<FdEventSink> fes;
        FswAbortIfNot(fd_channel.get_fd(fes), false);
        FswAbortIfNot(fes, false);
        FswMsgAbortOnErrno(
            ::connect(fes->get_fd(), to_sockaddr(&remote_addr),
                      sizeof(remote_addr)),
            false, 200, "Failed to connect to %s",
            address_to_string(to_sockaddr(&remote_addr), sizeof(remote_addr))
                .c_str());
        return true;
    }
    /**
     * Handle a new timer event and attempt to open a new socket.
     *
     * @param _timer The ExponentialBackoffTimer.
     *
     * @return True on success.
     */
    bool UdpConnection::handle_timer(ExponentialBackoffTimer &_timer)
    {
        FswAbortIfNot(timer == &_timer, false);
        if (is_closed())
        {
            FswAbortIfNot(timer->disable(), false);
            return true;
        }
        if (!is_connected())
        {
            /*
             * Try to re-open the socket.
             */
            if (!open_socket())
            {
                return true;
            }
            /*
             * Try to call ::connect() on the socket if that was requested. If
             * the connect fails, close the underlying channel so the socket is
             * not left in some half-configured state.
             */
            if (connected_socket)
            {
                if (!connect_socket())
                {
                    FswIfNot(fd_channel.close());
                    return true;
                }
            }
        }
        if (!signal_connect())
        {
            FswIfNot(close());
        }
        FswAbortIfNot(timer->disable(), false);
        return true;
    }
    /**
     * If a disconnect is received, start reconnecting.
     *
     * @param fes The FdEventSink.
     * @param fev The FdEvent.
     *
     * @return True on success.
     */
    bool UdpConnection::handle_fd_close(FdEventSink &fes, FdEvent &fev)
    {
        /**
         * The fd_channel's handle_fd_close function should have run
         * before us.
         */
        FswAssert(fd_channel.is_closed());
        if (!signal_connect())
        {
            FswAbortIfNot(close(), false);
            return true;
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
        /*
         * Start reconnecting, unless we were explicitly told to disconnect.
         */
        if (!is_disconnected_flag)
        {
            FswAbortIfNot(timer->enable(), false);
        }
        return true;
    }
    /**
     * Pass read events from the fd_channel up to clients.
     *
     * @param channel Must be fd_channel.
     *
     * @return True on success.
     */
    bool UdpConnection::handle_read(DgramChannel &channel)
    {
        FswAbortIfNot(&channel == &fd_channel, false);
        FswAbortIfNot(signal_read(), false);
        return true;
    }
    /**
     * Pass write events from the fd_channel up to clients.
     *
     * @param channel Must be fd_channel.
     *
     * @return True on success.
     */
    bool UdpConnection::handle_write(Channel &channel)
    {
        FswAbortIfNot(&channel == &fd_channel, false);
        FswAbortIfNot(signal_write(), false);
        return true;
    }
} /* end namespace Drone */