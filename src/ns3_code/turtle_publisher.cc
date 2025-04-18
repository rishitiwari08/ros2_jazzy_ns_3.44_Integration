#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/tap-bridge-module.h"

using namespace ns3;

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);

    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));

    NodeContainer nodes;
    nodes.Create(4);

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(5000000));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    NetDeviceContainer devices = csma.Install(nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper addresses;
    addresses.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = addresses.Assign(devices);

    // Tap bridge setup for external Linux host <-> node 0
    TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", StringValue("ConfigureLocal"));
    tapBridge.SetAttribute("DeviceName", StringValue("thetap"));
    tapBridge.Install(nodes.Get(0), devices.Get(0));

    // --- This is the UDP Server listening on node 1 ---
    uint16_t port = 12345;
    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(nodes.Get(1));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(60.0));
    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);

    Simulator::Run();
    Ptr<UdpServer> udpServer = serverApp.Get(0)->GetObject<UdpServer>();
    std::cout << "Received " << udpServer->GetReceived() << " packets." << std::endl;

    Simulator::Destroy();
    return 0;
}
