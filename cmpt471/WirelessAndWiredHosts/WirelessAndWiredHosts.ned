package inet.cmpt471.WirelessAndWiredHosts;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.node.wireless.AccessPoint;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.contract.IIntegratedVisualizer;
import ned.DatarateChannel;

network WirelessAndWiredHosts
{
    parameters:
        double per = default(0);
        int n;
    types:
        channel C extends DatarateChannel
        {
            datarate = 1Mbps;
            delay = 2ms;
            per = per;
        }
    submodules:
        visualizer: <default("IntegratedCanvasVisualizer")> like IIntegratedVisualizer if hasVisualizer() {
            parameters:
                @display("p=69,261;is=s");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=87,46;is=s");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            parameters:
                @display("p=85,161;is=s");
        }
        wirelessHost1[n]: WirelessHost {
            parameters:
                @display("p=100,100");
        }

        wiredHost2: StandardHost {
            parameters:
                @display("p=424,176");
        }
        router: Router {
            parameters:
                @display("p=340,161");
        }
        accessPoint: AccessPoint {
            parameters:
                @display("p=257,175");
        }
    connections:
        accessPoint.ethg++ <--> Eth100M <--> router.ethg++;
        wiredHost2.pppg++ <--> C <--> router.pppg++;
}
