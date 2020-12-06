#include <algorithm>

#include "inet/transportlayer/tcp/Tcp.h"
#include "inet/transportlayer/tcp/flavours/TcpVeno.h"

namespace inet {
namespace tcp {
Register_Class(TcpVeno);

TcpVenoStateVariables::TcpVenoStateVariables()
{
    ssthresh = 65535;
    //ssthresh = 0xFFFFFFFF;
    v_baseRTT = SIMTIME_MAX;
    v_minRTT = SIMTIME_MAX;
    v_sumRTT = 0.0;
    v_incr = 0;
    v_diff = 0;
    v_cntRTT = 0;
    v_beta = 6;
    v_begseq = 0;
    v_begtime = 0;
}

TcpVenoStateVariables::~TcpVenoStateVariables()
{
}

std::string TcpVenoStateVariables::str() const
{
    std::stringstream out;
    out << TcpBaseAlgStateVariables::str();
    out << " ssthresh=" << ssthresh;
    return out.str();
}

std::string TcpVenoStateVariables::detailedInfo() const
{
    std::stringstream out;
    out << TcpBaseAlgStateVariables::detailedInfo();
    out << "ssthresh = " << ssthresh << "\n";
    out << "baseRTT = " << v_baseRTT << "\n";
    return out.str();
}

TcpVeno::TcpVeno() : TcpReno(),
  state((TcpVenoStateVariables*&)TcpAlgorithm::state)
{
}

void TcpVeno::recalculateSlowStartThreshold()
{
  EV_INFO << "\nVeno recalculate ssthresh: v_diff = " << state->v_diff << ", snd_mss = " << state->snd_mss<< "\n";
  if (state->v_diff < state->v_beta * state->snd_mss) {
    // random loss due to bit errors is most likely to have occured
    // we cut down ssthres by 1/5 cwnd
    EV_INFO << "Random loss due to bit errors is mostly to have occured. Cutting down ssthresh by 1/5 of cwnd\n";
    double tmp = 4.0/5.0;
    uint32 flight_size = std::min(state->snd_cwnd, state->snd_wnd);  
    state->ssthresh = std::max(static_cast<uint32> (flight_size * tmp),
                              2 * state->snd_mss);
    EV_INFO << "New sshtresh = " << state->ssthresh << "\n";
    conn->emit(ssthreshSignal, state->ssthresh);
  } else {
    // congestion-based loss is most likely to have occured
    // we reduce cwnd by 1/2 as in Reno
    TcpReno::recalculateSlowStartThreshold();
  }
}

void TcpVeno::processRexmitTimer(TcpEventCode& event)
{
    TcpTahoeRenoFamily::processRexmitTimer(event);
    if (event == TCP_E_ABORT)
        return;

    recalculateSlowStartThreshold();
    // Same in Tcp Vegas: when rtx timeout: cwnd = 2*smss, instead of 1*smss (Reno)
    state->snd_cwnd = 2 * state->snd_mss;

    conn->emit(cwndSignal, state->snd_cwnd);
    EV_DETAIL << "RXT Timeout in Veno: resetting cwnd to " << state->snd_cwnd << "\n"
              << ", ssthresh=" << state->ssthresh << "\n";

    state->afterRto = true;


    conn->retransmitOneSegment(true);    //retransmit one segment from snd_una
}

void TcpVeno::receivedDataAck(uint32 firstSeqAcked)
{
  TcpTahoeRenoFamily::receivedDataAck(firstSeqAcked);
  const TcpSegmentTransmitInfoList::Item *found = state->regions.get(firstSeqAcked);
  if (found) {
    simtime_t currentTime = simTime();
    simtime_t tSent = found->getFirstSentTime();
    int num_transmit = found->getTransmitCount();

    simtime_t newRTT;
    if (state->v_cntRTT > 0) {
        newRTT = state->v_sumRTT / state->v_cntRTT;
        EV_DETAIL << "Veno: newRTT (state->v_sumRTT / state->v_cntRTT) calculated: " << state->v_sumRTT / state->v_cntRTT << "\n";
    }
    else {
        newRTT = currentTime - state->v_begtime;
        EV_DETAIL << "Veno: newRTT calculated: " << newRTT << "\n";
    }
    state->v_sumRTT = 0.0;
    state->v_cntRTT = 0;


    if (state->dupacks >= DUPTHRESH) {    // DUPTHRESH = 3
        //
        // Perform Fast Recovery: set cwnd to ssthresh (deflating the window).
        //
        EV_INFO << "Fast Recovery: setting cwnd to ssthresh=" << state->ssthresh << "\n";
        state->snd_cwnd = state->ssthresh;

        conn->emit(cwndSignal, state->snd_cwnd);
    }
    else {
        bool performSsCa = true; //perform slow start and congestion avoidance
        if (state && state->ect && state->gotEce) {
            // halve cwnd and reduce ssthresh and do not increase cwnd (rfc-3168, page 18):
            //   If the sender receives an ECN-Echo (ECE) ACK
            // packet (that is, an ACK packet with the ECN-Echo flag set in the TCP
            // header), then the sender knows that congestion was encountered in the
            // network on the path from the sender to the receiver.  The indication
            // of congestion should be treated just as a congestion loss in non-
            // ECN-Capable TCP. That is, the TCP source halves the congestion window
            // "cwnd" and reduces the slow start threshold "ssthresh".  The sending
            // TCP SHOULD NOT increase the congestion window in response to the
            // receipt of an ECN-Echo ACK packet.
            // ...
            //   The value of the congestion window is bounded below by a value of one MSS.
            // ...
            //   TCP should not react to congestion indications more than once every
            // window of data (or more loosely, more than once every round-trip
            // time). That is, the TCP sender's congestion window should be reduced
            // only once in response to a series of dropped and/or CE packets from a
            // single window of data.  In addition, the TCP source should not decrease
            // the slow-start threshold, ssthresh, if it has been decreased
            // within the last round trip time.
            if (simTime() - state->eceReactionTime > state->srtt) {
                state->ssthresh = state->snd_cwnd / 2;
                state->snd_cwnd = std::max(state->snd_cwnd / 2, uint32(1));
                state->sndCwr = true;
                performSsCa = false;
                EV_INFO
                               << "ssthresh = cwnd/2: received ECN-Echo ACK... new ssthresh = "
                               << state->ssthresh << "\n";
                EV_INFO << "cwnd /= 2: received ECN-Echo ACK... new cwnd = "
                               << state->snd_cwnd << "\n";

                // rfc-3168 page 18:
                // The sending TCP MUST reset the retransmit timer on receiving
                // the ECN-Echo packet when the congestion window is one.
                if (state->snd_cwnd == 1) {
                    restartRexmitTimer();
                    EV_INFO << "cwnd = 1... reset retransmit timer.\n";
                }
                state->eceReactionTime = simTime();
                conn->emit(cwndSignal, state->snd_cwnd);
                conn->emit(ssthreshSignal, state->ssthresh);
            }
            else
                EV_INFO << "multiple ECN-Echo ACKs in less than rtt... no ECN reaction\n";
            state->gotEce = false;
        }
        if (performSsCa) {

            
            // If ECN is not enabled or if ECN is enabled and received multiple ECE-Acks in
            // less than RTT, then perform slow start and congestion avoidance.

            if (state->snd_cwnd < state->ssthresh) {
                EV_INFO << "cwnd <= ssthresh: Slow Start: increasing cwnd by one SMSS bytes to ";

                // perform Slow Start. RFC 2581: "During slow start, a TCP increments cwnd
                // by at most SMSS bytes for each ACK received that acknowledges new data."
                state->snd_cwnd += state->snd_mss;

                conn->emit(cwndSignal, state->snd_cwnd);
                conn->emit(ssthreshSignal, state->ssthresh);

                EV_INFO << "cwnd=" << state->snd_cwnd << "\n";
            }
            else {
              EV_INFO << "Veno: newRTT = " << newRTT << "\n";
              if (newRTT > 0) {
                  uint32 rttLen = state->snd_nxt - state->v_begseq;
                  uint32 actual = rttLen / newRTT;
                  // expected = (current window size)/baseRTT
                  uint32 expected;
                  uint32 acked = state->snd_una - firstSeqAcked;
                  expected = (uint32)((state->snd_nxt - firstSeqAcked) + std::min(state->snd_mss - acked, (uint32)0)) / state->v_baseRTT;

                  // diff = (expected - actual) * baseRTT
                  uint32 diff = (uint32)((expected - actual) * SIMTIME_DBL(state->v_baseRTT) + 0.5);
                  state->v_diff = diff;
                  // Available bandwith is not fully utilized
                  // Increase cwnd by 1 every RTT
                  if (diff < state->v_beta * state->snd_mss) {
                      // perform Congestion Avoidance (RFC 2581)
                      EV_INFO << "Availabel bandwith is not fully utilized, increase cwnd by 1 every RTT\n";
                      uint32 incr = (state->snd_mss * state->snd_mss) / state->snd_cwnd;
                      if (incr == 0) incr = 1;
                      state->snd_cwnd += incr;
                      EV_INFO << "New cwnd = " << state->snd_cwnd << "\n";
                  } 
                  // Available bandwith is fully utilizied. 
                  // Increase cwnd by 1 every other RTT
                  else {
                      if (state->v_incr) {
                          EV_INFO << "Available bandwith is fully utilized, increase cwnd by 1 every other RTT\n";
                          state->v_incr = false;
                          // perform Congestion Avoidance (RFC 2581)
                          uint32 incr = state->snd_mss * state->snd_mss / state->snd_cwnd;
                          if (incr == 0) incr = 1;

                          state->snd_cwnd += incr;
                          EV_INFO << "New cwnd = " << state->snd_cwnd << "\n";
                      }else {
                          state->v_incr = true;
                      }
                  }

                  conn->emit(cwndSignal, state->snd_cwnd);
                  conn->emit(ssthreshSignal, state->ssthresh);

                  //
                  // Note: some implementations use extra additive constant mss / 8 here
                  // which is known to be incorrect (RFC 2581 p5)
                  //
                  // Note 2: RFC 3465 (experimental) "Appropriate Byte Counting" (ABC)
                  // would require maintaining a bytes_acked variable here which we don't do
                  //

                }
              // reset beqseq and begtime values for next rtt
              state->v_begseq = state->snd_nxt;
              state->v_begtime = currentTime;
            }
        }
        if (tSent != 0 && num_transmit == 1) {
            simtime_t newRTT = currentTime - tSent;
            state->v_sumRTT += newRTT;
            ++state->v_cntRTT;

            if (newRTT > 0) {
                if (newRTT < state->v_baseRTT)
                    state->v_baseRTT = newRTT;
            }
        }
    }
    // Same as Reno
    if (state->sack_enabled && state->lossRecovery) {
        // RFC 3517, page 7: "Once a TCP is in the loss recovery phase the following procedure MUST
        // be used for each arriving ACK:
        //
        // (A) An incoming cumulative ACK for a sequence number greater than
        // RecoveryPoint signals the end of loss recovery and the loss
        // recovery phase MUST be terminated.  Any information contained in
        // the scoreboard for sequence numbers greater than the new value of
        // HighACK SHOULD NOT be cleared when leaving the loss recovery
        // phase."
        if (seqGE(state->snd_una, state->recoveryPoint)) {
            EV_INFO << "Loss Recovery terminated.\n";
            state->lossRecovery = false;
        }
        // RFC 3517, page 7: "(B) Upon receipt of an ACK that does not cover RecoveryPoint the
        //following actions MUST be taken:
        //
        // (B.1) Use Update () to record the new SACK information conveyed
        // by the incoming ACK.
        //
        // (B.2) Use SetPipe () to re-calculate the number of octets still
        // in the network."
        else {
            // update of scoreboard (B.1) has already be done in readHeaderOptions()
            conn->setPipe();

            // RFC 3517, page 7: "(C) If cwnd - pipe >= 1 SMSS the sender SHOULD transmit one or more
            // segments as follows:"
            if (((int)state->snd_cwnd - (int)state->pipe) >= (int)state->snd_mss) // Note: Typecast needed to avoid prohibited transmissions
                conn->sendDataDuringLossRecoveryPhase(state->snd_cwnd);
        }
    }

  }
  state->regions.clearTo(state->snd_una);
  sendData(false);

}

void TcpVeno::receivedDuplicateAck()
{
    // Veno technique to handle duplicated acks is mostly the same as Reno
    // Only difference is how Veno retransmit the missing packet:
    //    if (v_diff < v_beta) /*random loss due to bit error*/ 
    //        ssthresh = cwnd * (4/5)
    //    else /*congestive loss*/
    //        ssthresh = cwnd * (1/2)
    //
    TcpTahoeRenoFamily::receivedDuplicateAck();

    state->regions.clearTo(state->snd_una);

    if (state->dupacks == DUPTHRESH) {    
        EV_INFO << "Veno on dupAcks == DUPTHRESH(=3): perform Fast Retransmit, and enter Fast Recovery:";

        if (state->sack_enabled) {
            if (state->recoveryPoint == 0 || seqGE(state->snd_una, state->recoveryPoint)) {    // HighACK = snd_una
                state->recoveryPoint = state->snd_max;    // HighData = snd_max
                state->lossRecovery = true;
                EV_DETAIL << " recoveryPoint=" << state->recoveryPoint;
            }
        }
        // enter Fast Recovery
        recalculateSlowStartThreshold(); // Compute ssthresh in Veno way
        // "set cwnd to ssthresh plus 3 * SMSS." (RFC 2581)
        state->snd_cwnd = state->ssthresh + 3 * state->snd_mss;    

        conn->emit(cwndSignal, state->snd_cwnd);

        EV_DETAIL << " set cwnd=" << state->snd_cwnd << ", ssthresh=" << state->ssthresh << "\n";

        // Fast Retransmission: retransmit missing segment without waiting
        // for the REXMIT timer to expire
        conn->retransmitOneSegment(false);
        
        if (state->sack_enabled) {
            // RFC 3517, page 7: "(4) Run SetPipe ()
            conn->setPipe();
            // RFC 3517, page 7: "(5) In order to take advantage of potential additional available
            // cwnd, proceed to step (C) below."
            if (state->lossRecovery) {
                EV_INFO << "Retransmission sent during recovery, restarting REXMIT timer.\n";
                restartRexmitTimer();

                // RFC 3517, page 7: "(C) If cwnd - pipe >= 1 SMSS the sender SHOULD transmit one or more
                // segments as follows:"
                if (((int)state->snd_cwnd - (int)state->pipe) >= (int)state->snd_mss) // Note: Typecast needed to avoid prohibited transmissions
                    conn->sendDataDuringLossRecoveryPhase(state->snd_cwnd);
            }
        }

        // try to transmit new segments (RFC 2581)
        sendData(false);
    }
    else if (state->dupacks > DUPTHRESH) {    
        //
        // Same as Reno: For each additional duplicate ACK received, increment cwnd by SMSS.
        // This artificially inflates the congestion window in order to reflect the
        // additional segment that has left the network
        //
        state->snd_cwnd += state->snd_mss;
        EV_DETAIL << "Veno on dupAcks > DUPTHRESH(=3): Fast Recovery: inflating cwnd by SMSS, new cwnd=" << state->snd_cwnd << "\n";

        conn->emit(cwndSignal, state->snd_cwnd);

        // RFC 3517, pages 7 and 8: "5.1 Retransmission Timeouts
        sendData(false);
    }
}

void TcpVeno::dataSent(uint32 fromseq)
{
    TcpReno::dataSent(fromseq);

    // save time when packet is sent
    // fromseq is the seq number of the 1st sent byte
    // we need this value, based on iss=0 (to store it the right way on the vector),
    // but iss is not a constant value (ej: iss=0), so it needs to be detemined each time
    // (this is why it is used: fromseq-state->iss)

    state->regions.clearTo(state->snd_una);
    state->regions.set(fromseq, state->snd_max, simTime());
}

void TcpVeno::segmentRetransmitted(uint32 fromseq, uint32 toseq)
{
    TcpReno::segmentRetransmitted(fromseq, toseq);

    state->regions.set(fromseq, toseq, simTime());
}
} // namespace tcp
} // namespace inet
