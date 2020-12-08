# CMPT471 Project - TCP Veno Implementation
Referenced paper: https://ieeexplore-ieee-org.proxy.lib.sfu.ca/document/1177186

TCP Veno Implementation source code: 
  - [TcpVeno.h](https://github.com/kutycoi123/inet/blob/cmpt471-project/src/inet/transportlayer/tcp/flavours/TcpVeno.h)
  - [TcpVeno.cc](https://github.com/kutycoi123/inet/blob/cmpt471-project/src/inet/transportlayer/tcp/flavours/TcpVeno.cc )

Networking topologies for testing TCP Veno:
  - [Multiple clients vs single server via a router](https://github.com/kutycoi123/inet/tree/cmpt471-project/cmpt471/MultipleClientsWithOneRouter)
  - [Wireless and wired hosts](https://github.com/kutycoi123/inet/tree/cmpt471-project/cmpt471/WirelessAndWiredHosts)

Known issues:
- It is very challenging for us to verify if our Tcp Veno implementation was implemented 100% correctly because this is a complex protocol and the implementation heavily 
depends on the framework that we used. In our case, we need to understand how OMNeT++ designs their complex simulation networking project and how Inet framework implements 
their protocols so that we can implement and intergrate TCP Veno with them. 
- So far, we have tested our implementation by observing the behavior of congestion window when we run testing topologies.

  In our topologies, the implementation gives expected result that correctly described how Tcp Veno works. 
- However, underlying bugs might still occur in edge cases, especially in highly congestive networks environment. 

VM image setup demo: https://youtu.be/YJ_BgZl7GLI

Topology demo: https://youtu.be/0AvadWsRlcA
