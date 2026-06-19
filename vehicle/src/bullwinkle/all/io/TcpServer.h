/**
 * @author Stefan Moluf
 * @date   06/02/11
 */
#ifndef TCP_SERVER_H
#define TCP_SERVER_H
#include "src/bullwinkle/all/FdBag.h"
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Server.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/io/FdStreamChannel.h"
namespace Drone
{
    /**
     * A Server which listens on a TCP port and spawns FdStreamChannels
     * for each new connection.
     */
    class TcpServer : public Server
    {
    public:
        typedef Signal<bool, TcpServer &, Handle<FdStreamChannel>> signal_t;
        TcpServer(FdBag &_fdbag, const size_t _size = 1500,
                  const uint _max_connections = UINT_MAX);
        virtual ~TcpServer();
        /**
         * Signal emitted when a new client connects.
         */
        signal_t connect_sig;

    protected:
        virtual bool server_create_connection(Handle<FdEventSink> fes,
                                              Handle<ServerFd> &connection);

    private:
        /**
         * Create new channels with this buffer size.
         */
        const size_t size;
    };
} /* end namespace Drone */
#endif /* TCP_SERVER_H */