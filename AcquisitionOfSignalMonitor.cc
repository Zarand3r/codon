/**
 * @author Mike Hoot
 * @date   2021-03-15
 */
#include "AcquisitionOfSignalMonitor.h"
namespace Drone
{
    /**
     * Create and initialize an AcquisitionOfSignalMonitor
     *
     * @param builder: SlateBuilder to create slate variables.
     * @param aos_threshold: Threshold for detecting rising edge of Acquisition
     *                       of Signal. We detect AOS if
     *                       bidirectional_comms_age < aos_threshold. Defaults
     * to one second.
     *
     * @return A ready-to-use AcquisitionOfSignalMonitor or NULL on error.
     */
    Handle<AcquisitionOfSignalMonitor>
    AcquisitionOfSignalMonitor::create(SlateBuilder &builder,
                                       const nano_t aos_threshold)
    {
        Handle<AcquisitionOfSignalMonitor> empty;
        Handle<AcquisitionOfSignalMonitor> aos_monitor(
            new AcquisitionOfSignalMonitor(aos_threshold));
        SacAbortIfNot(aos_monitor, empty);
        SacAbortIfNot(aos_monitor->init(builder), empty);
        return aos_monitor;
    }
    /**
     * Initialize an AcquisitionOfSignalMonitor.
     *
     * @param builder: SlateBuilder to create slate variables.
     *
     * @returns true if AcquisitionOfSignalMonitor was successfully initialized,
     * false otherwise.
     */
    bool AcquisitionOfSignalMonitor::init(SlateBuilder &builder)
    {
        SacAbortIfNot(
            builder.bind("loss_of_comm_fdir.bidirectional_comm_age_ns",
                         bidirectional_comm_age_tok),
            false);
        SacAbortIfNot(builder.bind("out_of_contact_threshold",
                                   out_of_contact_threshold_tok),
                      false);
        SacAbortIfNot(builder.create("retry_on_connection", false, shard_sync,
                                     slate_read_only, retry_on_connection_tok),
                      false);
        slate = builder.slate(slate_no_validation);
        return true;
    }
    /*
     * Dispatch the monitor. Checks if we've just reestablished connectivity,
     * and if so emits a signal to retry in-progress GRPC calls.
     *
     * @param control_time Current control_time.
     *
     * @return Always nano_t_max.
     */
    nano_t AcquisitionOfSignalMonitor::dispatch(nano_t control_time)
    {
        /*
         * Send a signal to kill and retry GRPC calls if and only if:
         * 1. We've reestablished connections recently (< aos_threshold)
         * 2. The retry_on_connection token is set, indicating that we were
         *    out of contact > out_of_contact_threshold.
         */
        if (slate[bidirectional_comm_age_tok] < aos_threshold &&
            slate[retry_on_connection_tok])
        {
            dbnprintf(
                200,
                "Re-acquired signal (bidirectional_comm_age = %f seconds),"
                " killing and retrying any in-progress GRPCs\n",
                to_seconds(slate[bidirectional_comm_age_tok]));
            retry_grpc_sig.emit();
        }
        /**
         * Prepare to emit the signal next AOS if we've been out of contact for
         * longer than the out_of_contact_threshold.
         *
         * If the threshold is 0, don't do anything (setting the threshold to 0
         * effectively disables the component)
         */
        const bool disabled = slate[out_of_contact_threshold_tok] <= 0;
        slate[retry_on_connection_tok] =
            !disabled && (slate[bidirectional_comm_age_tok] >
                          to_nano_t(slate[out_of_contact_threshold_tok]));
        return nano_t_max;
    }
    /*
     * Constructor.
     *
     * @param aos_threshold: Threshold for detecting rising edge of Acquisition
     *                       of Signal. We detect AOS if
     *                       bidirectional_comms_age < aos_threshold.
     */
    AcquisitionOfSignalMonitor::AcquisitionOfSignalMonitor(
        const nano_t _aos_threshold)
        : aos_threshold(_aos_threshold), slate(), bidirectional_comm_age_tok(),
          out_of_contact_threshold_tok(), retry_on_connection_tok()
    {}
} // namespace Drone