/**
 * @author Stefan Moluf
 * @date   01/24/11
 */
#ifndef FD_DGRAM_CHANNEL_H
#define FD_DGRAM_CHANNEL_H
#include "src/bullwinkle/all/BipBuffer.h"
#include "src/bullwinkle/all/FdEventSink.h"
#include "src/bullwinkle/all/io/DgramChannel.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>
namespace Drone
{
    /**
     * Specifies semantics of a particular datagram socket.
     */
    enum dgram_fd_type_t
    {
        /**
         * No semantics.
         */
        dgram_fd_none,
        /**
         * Cannot use sockaddrs. Must use read() not recvfrom(). Zero-length
         * reads indicate EOF.
         */
        dgram_fd_non_socket,
        /**
         * Cannot use sockaddrs. Must use read() not recvfrom(). Zero-length
         * reads acceptable.
         */
        dgram_fd_device_file,
        /**
         * Cannot use sockaddrs. Can use recvfrom(). Zero-length reads
         * acceptable.
         */
        dgram_fd_conn_socket,
        /**
         * Must use sockaddrs. Can use recvfrom(). Zero-length reads
         * acceptable.
         */
        dgram_fd_uconn_socket,
        /**
         * A Tun device cannot use sockaddrs. Must use read() not recvfrom().
         * Tun devices can return EINVAL on bad packets, and those should be
         * dropped instead of causing an error that closes the fd and channel.
         */
        dgram_fd_tun
    };
    /**
     * A duplex DgramChannel backed by a datagram-oriented file
     * descriptor.
     *
     * Data is read from the file descriptor into an internal buffer, at
     * which point read_sig is emitted. Writes are similarly buffered until
     * the underlying fd emits a write event, at which point they are flushed
     * and write_sig is emitted.
     *
     * Write signals are emitted from the FdDgramChannel when any space
     * becomes available in the write buffer. Read signals are emitted
     * whenever there is new data, or if the fd emits a read event and
     * the read buffer is full.
     *
     * If the FdDgramChannel is placed into a mode which requires sockaddrs,
     * incoming addresses will be recorded and stored inline with the
     * datagrams. These can be retrieved with the get_address() method.
     *
     * Note that calling close() on an FdDgramChannel will not close the
     * underlying file descriptor until all data from the input buffer
     * has been drained. No more data will be read from the file descriptor
     * during this period.
     *
     * FdDgramChannels are slightly unusual in that they can be re-opened
     * even after their close signal has been emitted by calling
     * assign_fd() again. Because file descriptors can close during normal
     * operations, this ability avoids the need to reallocate an entirely
     * new FdDgramChannel at runtime. Note that as with other Channels,
     * all signals are cleared when an FdDgramChannel closes.
     */
    class FdDgramChannel : public DgramChannel
    {
    public:
        /**
         * A Signal that takes the address of an incoming dgram and returns
         * whether the dgram should be allowed.
         */
        using firewall_signal_t =
            Signal<bool, const sockaddr &, const socklen_t &>;
        /**
         * A Signal that takes the contents of a dgram that resulted
         * in a write error (along with the errno value), and returns
         * whether the error should be ignored. (Error is not ignored
         * if the Signal is empty().)
         */
        using write_error_signal_t = Signal<bool, const B2c &, int>;
        FdDgramChannel();
        FdDgramChannel(const size_t min_num_dgrams, const size_t max_dgram_len);
        FdDgramChannel(const size_t min_num_input_dgrams,
                       const size_t _max_input_dgram_len,
                       const size_t min_num_output_dgrams,
                       const size_t _max_output_dgram_len);
        virtual ~FdDgramChannel();
        virtual bool assign_fd(Handle<FdEventSink> _fes,
                               const dgram_fd_type_t _fd_type,
                               const bool is_nonblocking = false,
                               const std::string &mode = "rw");
        virtual bool get_fd(Handle<FdEventSink> &_fes) const;
        virtual bool get_address(sockaddr *const addr,
                                 socklen_t *const addr_len) const;
        bool can_read() const;
        bool can_write() const;
        /**
         * Toggles "low-latency mode" for writes to the fd. When enabled, the
         * behavior of FdDgramChannel in the write direction changes to:
         *
         *   - Only show space available if writes to the underlying fd
         *       would not block.
         *   - Allow a maximum of one datagram written per call to select()
         *       processed by the underlying FdEventSink.
         *
         * Together, these constraints mean that any writes to the Channel
         * will be immediately be flushed to the fd and will not remain in
         * the buffer. The disadvantage of low-latency mode is that CPU usage
         * may be higher and it will not work correctly in event loops where
         * select() is not dispatched continuously.
         *
         * @param low_latency True to enable low-latency mode for this
         *                    Channel. False to disable it.
         */
        void set_low_latency(bool low_latency) { is_low_latency = low_latency; }
        /**
         * Controls the behavior where the channel only keeps the last datagram
         * received after each read.
         *
         * @param keep_only_last_dgram Whether to drop all but the last datagram
         *                             on reads.
         */
        void set_keep_only_last_dgram(bool _keep_only_last_dgram)
        {
            keep_only_last_dgram = _keep_only_last_dgram;
        }
        /**
         * Set flags arg for the call to sendto().
         *
         * @param flags Bitwise OR of flags for sendto() syscall.
         */
        void set_send_flags(int flags) { send_flags = flags; }
        /**
         * Ignore known write errors which could indicate transient issues.
         */
        void set_ignore_write_errors(bool ignore)
        {
            ignore_write_errors = ignore;
        }
        /*
         * Non-Channel functions. These enable the use of unconnected sockets
         * over a familiar interface, but can't be directly plumbed into
         * other Channel-oriented classes.
         */
        virtual bool commit_dataframe_to(DataFrame &frame,
                                         const sockaddr *const addr,
                                         const socklen_t addr_len);
        virtual bool write_to(const char *data, const size_t data_len,
                              const sockaddr *const addr,
                              const socklen_t addr_len);
        virtual bool write_to(const B2c &data, const sockaddr *const addr,
                              const socklen_t addr_len);
        /*
         * See DgramChannel for complete API.
         */
        virtual bool clear() override;
        virtual bool is_closed() const override;
        virtual bool is_empty() const override;
        virtual bool is_drained() const override;
        virtual size_t space_left() const override;
        virtual DataFrame &get_dataframe(const size_t request_len) override;
        virtual bool commit_dataframe(DataFrame &frame) override;
        virtual uint dgrams_avail() const override;
        virtual B2c peek_dgram() const override;
        uint dgrams_unwritten() const;
        bool clear_unwritten();
        /*
         * A signal called for every incoming socket dgram. If false is
         * returned, the dgram will be dropped.
         */
        firewall_signal_t firewall_signal;
        /**
         * Optional Signal for handling write() errors.
         *
         * This is emitted _from the write handler_ -- beware
         * reentrancy issues!
         *
         * If empty, no change in behavior.
         *
         * Each handler will be called with the dgram contents that
         * resulted in the error along with the error (errno) value.
         *
         * If non-empty and the last handler returns false, then we will
         * continue with the error path.
         *
         * Otherwise, the dgram will be dropped and processing will
         * continue.
         */
        write_error_signal_t write_error_signal;

    protected:
        virtual bool clear_signals() override;
        virtual bool pop_dgram() override;
        virtual bool channel_commit_dataframe(DataFrame &frame) override;
        virtual bool channel_close() override;
        virtual bool channel_clear() override;
        virtual bool channel_pop_dgram() override;
        virtual bool handle_fd_read(FdEventSink &_fes, FdEvent &fev);
        virtual bool handle_fd_write(FdEventSink &_fes, FdEvent &fev);
        virtual bool handle_fd_close(FdEventSink &_fes, FdEvent &fev);
        /**
         * Datagrams written to this buffer by clients are written into
         * the file descriptor.
         */
        BipBufferHeap input;
        /**
         * Writes data from the file descriptor into this buffer for
         * reading by clients.
         */
        BipBufferHeap output;
        /**
         * The maximum input datagram size allowed.
         */
        const size_t max_input_dgram_len;
        /**
         * The maximum output datagram size allowed.
         */
        const size_t max_output_dgram_len;
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
        /**
         * True if the file descriptor is nonblocking, and multiple reads
         * and writes per event are allowed.
         */
        bool is_nonblocking_flag;
        /**
         * The type of the active file descriptor.
         */
        dgram_fd_type_t fd_type;
        /**
         * True during periods of time when handle_fd_write() is in the middle
         * of a call.
         */
        bool handling_write;
        /**
         * True when low-latency mode is enabled for the write direction of
         * this Channel. See set_low_latency() for more information.
         */
        bool is_low_latency;
        /**
         * True if known write errors that could indicate a transient issue
         * should be ignored.
         */
        bool ignore_write_errors;
        /**
         * True if we only want to keep the last datagram received in the
         * read buffer.
         */
        bool keep_only_last_dgram;
        /**
         * Flags argument for the call to sendto().
         */
        int send_flags;
    };
} /* end namespace Drone */
#endif /* FD_DGRAM_CHANNEL_H */