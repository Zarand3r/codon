/**
 * @author Stefan Moluf
 * @date   04/09/11
 */
#ifndef STREAM_CONNECTION_H
#define STREAM_CONNECTION_H
#include "src/bullwinkle/all/io/StreamChannel.h"
namespace Drone
{
    /**
     * A duplex StreamChannel which may have intermittent connectivity.
     */
    class StreamConnection : public StreamChannel
    {
    public:
        typedef Signal<bool, StreamConnection &> signal_t;
        StreamConnection();
        virtual ~StreamConnection();
        /**
         * Returns true if the channel is connected and receiving data.
         *
         * @return True if the channel is connected and receiving data.
         */
        virtual bool is_connected() const = 0;
        /**
         * Disconnect this channel without closing it.
         *
         * @return True on success.
         */
        virtual bool disconnect() = 0;
        /**
         * Terminate the currently active connection and allow it to
         * automatically reconnect.
         *
         * @return True on success.
         */
        virtual bool reset() = 0;
        /**
         * Indicate whether the connection is or isn't attempting to connect.
         *
         * @return True on success.
         */
        virtual bool is_enabled() const = 0;
        virtual bool pop_front(const size_t bytes);
        /**
         * Return the space currently taken up by writes into the channel
         * (i.e., the amount of data yet to be flushed to the underlying file
         * descriptor).
         *
         * @return The space currently taken up by writes into the channel.
         */
        virtual size_t space_taken() const = 0;
        virtual bool commit_dataframe(DataFrame &frame);
        /**
         * Signal emitted when the status of the connection changes.
         *
         * Returning false from this signal will cause the
         * StreamConnection to close.
         */
        signal_t connect_sig;

    protected:
        virtual bool signal_read();
        virtual bool signal_connect();
        virtual bool clear_signals();
    };
} /* end namespace Drone */
#endif /* STREAM_CONNECTION_H */