//
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

package inet.cmpt471.MultipleClientsWithOneRouter;

import inet.common.misc.ThruputMeteringChannel;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import ned.DatarateChannel;


network MultipleClientsServer
{
    parameters:
        int n;
        double per = default(0);
    types:
        channel C extends DatarateChannel
        {
			datarate = 2Mbps;
			delay = 2ms;
            per = per;
        }
    submodules:
        configurator: Ipv4NetworkConfigurator {
            parameters:
                @display("p=100,100;is=s");
        }
        r1: Router {
            @display("p=435,127");
        }
        cli[n]: StandardHost {
            parameters:
                @display("i=device/laptop;p=250,100,col,100");
        }
        srv: StandardHost {
            parameters:
                @display("i=device/server;p=624,124");
        }
    connections:
        for i=0..n-1 {
            cli[i].pppg++ <--> C <--> r1.pppg++;
        }
        r1.pppg++ <--> C <--> srv.pppg++;
}

