/**
 * @author Stefan Moluf
 * @date   01/24/11
 */
#ifndef FD_STREAM_CHANNEL_H
#define FD_STREAM_CHANNEL_H
#include "src/bullwinkle/all/DataQueue.h"
#include "src/bullwinkle/all/FdEventSink.h"
#include "src/bullwinkle/all/io/StreamChannel.h"
namespace Drone
{
    /**
     * A duplex StreamChannel backed by a stream-oriented file
     * descriptor.
     *
     * Data is read from the file descriptor into an internal buffer, at
     * which point read_sig is emitted. Writes are similarly buffered until
     * the underlying fd emits a write event, at which point they are flushed
     * and write_sig is emitted.
     *
     * Write signals are only emitted from the FdStreamChannel when its
     * buffer is empty in order to cut down on data shifting. Read signals
     * are emitted whenever there is new data, or if the fd emits a read
     * event and the read buffer is full.
     *
     * Note that calling close() on an FdStreamChannel will not close the
     * underlying file descriptor until all data from the input buffer
     * has been drained. No more data will be read from the file descriptor
     * during this period.
     *
     * FdStreamChannels are slightly unusual in that they can be re-opened
     * even after their close signal has been emitted by calling
     * assign_fd() again. Because file descriptors can close during normal
     * operations, this ability avoids the need to reallocate an entirely
     * new FdStreamChannel at runtime. Note that as with other Channels,
     * all signals are cleared when an FdStreamChannel closes.
     */
    class FdStreamChannel : public StreamChannel
    {
    public:
        FdStreamChannel();
        explicit FdStreamChannel(const size_t size);
        virtual ~FdStreamChannel();
        virtual bool assign_fd(Handle<FdEventSink> _fes,
                               const std::string &mode = "rw");
        virtual bool get_fd(Handle<FdEventSink> &_fes) const;
        bool can_read() const;
        bool can_write() const;
        /*
         * See StreamChannel for complete API.
         */
        virtual bool clear();
        virtual bool is_closed() const;
        virtual bool is_empty() const;
        virtual bool is_drained() const;
        virtual size_t get_max_data_len() const;
        virtual size_t space_left() const;
        virtual size_t space_taken() const;
        virtual DataFrame &get_dataframe(const size_t request_len);
        virtual B2c get_data() const;
        virtual bool pop_front(const size_t bytes);

    protected:
        virtual bool channel_clear();
        virtual bool channel_close();
        virtual bool channel_commit_dataframe(DataFrame &frame);
        virtual bool channel_pop_front(const size_t bytes);
        virtual bool handle_fd_read(FdEventSink &_fes, FdEvent &fev);
        virtual bool handle_fd_write(FdEventSink &_fes, FdEvent &fev);
        virtual bool handle_fd_close(FdEventSink &_fes, FdEvent &fev);
        /**
         * Data written to this buffer by clients is written into
         * the file descriptor.
         */
        DataQueueHeap input;
        /**
         * Writes data from the file descriptor into this buffer for
         * reading by clients.
         */
        DataQueueHeap output;
        /**
         * True if the fd was opened with read mode set.
         */
        bool can_read_flag;
        /**
         * True if the fd was opened with write mode set.
         */
        bool can_write_flag;
        /**
         * The active file descriptor.
         */
        Handle<FdEventSink> fes;
        /**
         * True if close() has been called with this file descriptor.
         */
        bool is_closed_flag;
    };
} /* end namespace Drone */
#endif /* FD_STREAM_CHANNEL_H */