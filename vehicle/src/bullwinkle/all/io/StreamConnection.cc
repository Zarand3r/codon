/**
 * @author Stefan Moluf
 * @date   04/09/11
 */
#include "src/bullwinkle/all/io/StreamConnection.h"
#include "src/bullwinkle/all/core/sac.h"
namespace Drone
{
    StreamConnection::StreamConnection() : connect_sig() {}
    StreamConnection::~StreamConnection() {}
    /**
     * Emit the connect_sig and return the result.
     *
     * @return True on success.
     */
    bool StreamConnection::signal_connect()
    {
        if (connect_sig.empty())
            return true;
        return connect_sig.emit(*this);
    }
    /*
     * See StreamChannel.
     */
    bool StreamConnection::clear_signals()
    {
        connect_sig.clear();
        SacAbortIfNot(StreamChannel::clear_signals(), false);
        return true;
    }
    /**
     * Remove bytes from the front of the channel. Overridden to supress
     * write signals.
     *
     * @param bytes Pop this many bytes.
     *
     * @return True if all requested bytes were removed. False on
     * error. If there were insufficient bytes, no bytes will
     * be removed.
     */
    bool StreamConnection::pop_front(const size_t bytes)
    {
        if (0 == bytes)
            return true;
        SacAbortIf(is_empty(), false);
        SacAbortIfNot(channel_pop_front(bytes), false);
        /*
         * If we have just drained the channel, emit a close signal.
         */
        if (is_drained())
        {
            /*
             * There's not a lot we can do if this fails. We've already
             * successfully closed the channel and popped the data.
             */
            SacIfNot(signal_close());
        }
        return true;
    }
    /*
     * See Channel. Overridden to suppress read signals.
     */
    bool StreamConnection::commit_dataframe(DataFrame &frame)
    {
        SacAbortIf(is_closed(), false);
        SacAbortIfNot(channel_commit_dataframe(frame), false);
        return true;
    }
    /**
     * Emit the read signal and return the result.
     *
     * Does not close the channel if the read handler returns false - the
     * course of action is left to the caller of signal_read().
     *
     * @return Result.
     */
    bool StreamConnection::signal_read()
    {
        if (read_sig.empty())
            return true;
        while (!is_empty())
        {
            has_popped = false;
            if (SacIfNot(read_sig.emit(*this)))
                return false;
            if (!has_popped)
                break;
        }
        return true;
    }
} /* end namespace Drone */