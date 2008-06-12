
# from msg files
my $arglessGetters = "fec addr destAddr destAddress nextHopAddr receiverAddress
    senderAddress srcAddr srcAddress recordRoute
    sourceRoutingOption timestampOption destAddr destAddress
    destinationAddress prefix srcAddr srcAddress targetAddress
    destAddr localAddr remoteAddr
    srcAddr sourceLinkLayerAddress targetLinkLayerAddress abit
    ackBit autoAddressConfFlag serverClose dbit dontFragment finBit
    fin_ack_rcvd fork isRequest isWithdraw managedAddrConfFlag
    moreFragments onlinkFlag otherStatefulConfFlag overrideFlag pshBit
    rbit routerFlag rstBit solicitedFlag synBit tbit urgBit msg
    diffServCodePoint routingType segmentsLeft bitrate holdTime
    keepAliveTime replyDelay MTU ackNo channelNumber code connId
    curHopLimit destPort endSequenceNo errorCode expectedReplyLength flag
    flowLabel fragmentOffset identification identifier interfaceId irs
    iss label lsaLength localPort optionCode payloadLength preferredLifetime
    prefixLength protocol pvLim rcv_nxt rcv_up rcv_wnd reachableTime
    remotePort retransTimer seqNumber sequenceNo snd_max snd_mss snd_nxt
    snd_una snd_up snd_wl1 snd_wl2 snd_wnd sockId srcPort state status
    trafficClass transportProtocol type userId validLifetime originatorId
    seqNo urgentPointer window destPort fragmentOffset headerLength
    hopLimit lastAddressPtr nextAddressPtr overflow protocol
    routerLifetime srcPort timeToLive version
    family messageText receiveQueueClass receiverLDPIdentifier
    sendQueueClass stateName tcpAlgorithmClass";

# array fields in msg files
my $gettersWithArg = "payload recordAddress address extensionHeader
    prefixInformation recordTimestamp addresses data";

foreach $i (split(/\s/, $gettersWithArg)) {$arglessGetters .= " ${i}ArraySize";}

$arglessGetters =~ s/\s+/|/g;
$gettersWithArg =~ s/\s+/|/g;

$listfname = $ARGV[0];
open(LISTFILE, $listfname) || die "cannot open $listfname";
while (<LISTFILE>)
{
    chomp;
    s/\r$//; # cygwin/mingw perl does not do CR/LF translation

    $fname = $_;

    if ($fname =~ /_m\./) {
        print "skipping $fname...\n";
        next;
    }

    print "processing $fname... ";

    open(INFILE, $fname) || die "cannot open $fname";
    read(INFILE, $txt, 1000000) || die "cannot read $fname";
    close INFILE;

    my $origtxt = $txt;

    # process $txt:

    # remove omitGetVerb from .msg files
    $txt =~ s/\n *\@omitGetVerb\(true\); *\n/\n/gs;

    # rename getters
    $txt =~ s/\b($arglessGetters) ?\( *\)/"get".ucfirst($1)."()"/mge;
    $txt =~ s/\b($gettersWithArg) ?\(/"get".ucfirst($1)."("/mge;

    if ($txt eq $origtxt) {
        print "unchanged\n";
    } else {
        open(OUTFILE, ">$fname") || die "cannot open $fname for write";
        print OUTFILE $txt || die "cannot write $fname";
        close OUTFILE;
        print "DONE\n";
    }
}

print "\nConversion done. You may safely re-run this script as many times as you want.\n";

