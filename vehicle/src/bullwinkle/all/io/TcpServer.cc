/**
 * @author Stefan Moluf
 * @date   06/02/11
 */
#include "src/bullwinkle/all/io/TcpServer.h"
#include "src/bullwinkle/all/io/FdStreamChannel.h"
#include "src/bullwinkle/all/io/TcpServerFd.h"
#include "src/bullwinkle/all/sock_util.h"
namespace Drone
{
    /**
     * Construct a TcpServer.
     *
     * @param _fdbag The FdBag to register file descriptors with.
     * @param _size Create new FdStreamChannels with buffers of this
     *              size.
     * @param _max_connections Allow this many simultaneous
     *                        connections to the TcpServer.
     */
    TcpServer::TcpServer(FdBag &_fdbag, const size_t _size,
                         const uint _max_connections)
        : Server(_fdbag, _max_connections), size(_size)
    {}
    /**
     * TcpServer destructor.
     */
    TcpServer::~TcpServer() {}
    /**
     * Handle a new connection by creating a new FdStreamChannel and
     * emitting it.
     *
     * @param      fes Wrap this FdEventSink in an FdStreamChannel.
     * @param[out] conn Returns a TcpServerFd with the FdStreamChannel
     *                  inside.
     *
     * @return True on success.
     */
    bool TcpServer::server_create_connection(Handle<FdEventSink> fes,
                                             Handle<ServerFd> &conn)
    {
        FswAbortIfNot(fes, false);
        FswAbortIf(fes->is_closed(), false);
        /*
         * Enable nodelay to improve responsiveness.
         */
        FswAbortIfNot(set_tcp_no_delay(fes->get_fd(), true), false);
        /*
         * Enable TCP keepalive timers to prevent stale server connections.
         */
        const int keepalive_probe_interval = 1;
        const int keepalive_retry_interval = 1;
        const int keepalive_retry_count = 10;
        FswAbortIfNot(Drone::enable_tcp_keepalive(
                          fes->get_fd(), keepalive_probe_interval,
                          keepalive_retry_interval, keepalive_retry_count),
                      false);
        /*
         * Create a Channel to manage this client.
         */
        Handle<FdStreamChannel> channel(new FdStreamChannel(size));
        FswAbortIfNot(channel, false);
        FswAbortIfNot(channel->assign_fd(fes), false);
        /*
         * Emit the new channel to clients. If they reject it, just
         * return.
         */
        FswAbortIf(connect_sig.empty(), false);
        FswAbortIfNot(connect_sig.emit(*this, channel), false);
        /*
         * Create TcpServerFd so that the Server can hold on to the
         * Channel.
         */
        Handle<TcpServerFd> new_conn(new TcpServerFd(fes, channel));
        FswAbortIfNot(new_conn, false);
        conn = new_conn;
        return true;
    }
} /* end namespace Drone */