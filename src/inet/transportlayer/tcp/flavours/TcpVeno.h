#ifndef __INET_TCPVENO_H
#define __INET_TCPVENO_H
#include "inet/transportlayer/tcp/flavours/TcpBaseAlg.h"
#include "inet/transportlayer/tcp/flavours/TcpReno.h"
#include "inet/transportlayer/tcp/flavours/TcpTahoeRenoFamily.h"
#include "inet/transportlayer/tcp/flavours/TcpSegmentTransmitInfoList.h"

namespace inet {
namespace tcp {
/*
 * State variables for TcpVeno
 *
 */
class INET_API TcpVenoStateVariables : public TcpBaseAlgStateVariables
{
  public:
    TcpVenoStateVariables();
    ~TcpVenoStateVariables();
    virtual std::string str() const override;
    virtual std::string detailedInfo() const override;

    simtime_t v_baseRTT;
    simtime_t v_minRTT;
    simtime_t v_sumRTT;    // sum of rtt's measured within one RTT
    uint32 v_diff; // Difference bewteen expected and actual throughput
    uint32 v_beta;
    bool v_incr; // If true, send window needs to be increamented
    int v_cntRTT;    // # of rtt's measured within one RTT
    uint32 ssthresh;

    uint32 v_begseq;    // register next pkt to be sent,for rtt calculation in receivedDataAck
    simtime_t v_begtime;    // register time for rtt calculation

    //simtime_t v_rtt_timeout;    // vegas fine-grained timeout
    //simtime_t v_sa;    // average for vegas fine-grained timeout
    //simtime_t v_sd;    // deviation for vegas fine-grained timeout

    TcpSegmentTransmitInfoList regions;
};

class INET_API TcpVeno : public TcpReno {
  protected:
    TcpVenoStateVariables *& state;
    /** Create and return a TCPVenoStateVariables object. */
    virtual TcpStateVariables *createStateVariables() override
    {
        return new TcpVenoStateVariables();
    }
    /** Utility function to recalculate ssthresh */
    virtual void recalculateSlowStartThreshold();

    /** Redefine what should happen on retransmission */
    virtual void processRexmitTimer(TcpEventCode& event) override;

  public:
    /** Ctor */
    TcpVeno();

    /** Redefine what should happen when data got acked, to add congestion window management */
    virtual void receivedDataAck(uint32 firstSeqAcked) override;

    /** Redefine what should happen when dupAck was received, to add congestion window management */
    virtual void receivedDuplicateAck() override;

    /** Called after we send data */
    virtual void dataSent(uint32 fromseq) override;

    virtual void segmentRetransmitted(uint32 fromseq, uint32 toseq) override;


};

} // namespace tcp
} // namespace inet
#endif
