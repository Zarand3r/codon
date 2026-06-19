/**
 * @author Mike Hoot
 * @date   2021-03-15
 */
#ifndef ACQUISISTION_OF_SIGNAL_MONITOR_H
#define ACQUISISTION_OF_SIGNAL_MONITOR_H
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateBuilder.h"
namespace Drone
{
    /*
     * Monitors when the bidirectional comms flag goes high and emits a signal
     * on acquisition of signal. Useful to trigger slow polling gRPCs when
     * bidirectional comms is observed.
     */
    class AcquisitionOfSignalMonitor : public SignalHandler
    {
    public:
        static Handle<AcquisitionOfSignalMonitor>
        create(SlateBuilder &builder, const nano_t _aos_threshold = billion);
        nano_t dispatch(nano_t control_time);
        /*
         * Signal to kill and retry any outstanding GRPC calls.
         *
         * @return Always true
         */
        Signal<bool> retry_grpc_sig;

    private:
        AcquisitionOfSignalMonitor(const nano_t _out_of_contact_threshold);
        bool init(SlateBuilder &builder);
        /*
         * Threshold to determine if we've acquired signal.
         *
         * We've acquired signal iff bidirectional_comm_age < aos_threshold.
         */
        const nano_t aos_threshold;
        /*
         * Slate containing the following tokens.
         */
        Slate slate;
        /*
         * Whether we have bidirectional comm with Satops.
         */
        ReadToken<nano_t> bidirectional_comm_age_tok;
        /*
         * How long we should be out of contact before killing and retrying
         * GRPCs on reconnection. If set to zero, the monitor will be disabled.
         */
        ReadToken<double> out_of_contact_threshold_tok;
        /*
         * Whether we should attempt to retry our GRPCs on the next AOS.
         */
        WriteToken<bool> retry_on_connection_tok;
    };
} /* end namespace Drone */
#endif /* ACQUISITION_OF_SIGNAL_MONITOR */
