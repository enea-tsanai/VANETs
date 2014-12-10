
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* This script implements the Vanet simulation for Routing Evaluation with the presence of buildings
 * ToDo or check:
 * 1) Bs vs STA range
 * 2) Total number of simtime, Packets and interval
 * 3) Data Rates in Phymode
 * 4) Number of Vehicles
 */

/*
 * Author: ENEA TSANAI
 * e-mail: tsanai@ceid.upatras.gr, tsanaienea@gmail.com
*/

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/gpsr-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/energy-module.h"
#include "ns3/buildings-module.h"

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

// Default Network Topology
//    *                     * * **    *
//        *  *  *          **         *
//                                                   
//          **         *               WIFI (300m)     +------------+
// *  *  *    VANET (WIFI 300m)    << - - - - -      |Base Station| ==((*))
//    *                       *                        +------------+
//    *                     * * **    *
//        **         * *  *  * 


using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("Vanets_and_Apps");

/* Num of vehicles has to be global to init array NodeTotEnergyCons[]..
 * Else switch to vector*/
const uint32_t veh_num = 2; // num of vehicles
/* Log files */
std::string outdir = "logs/";
std::string log_1 = "energy_logs.txt";
std::string log_2 = "mobility_logs.txt";
std::string log_3 = "BS_logs.txt";
std::string log_4 = "STA_logs.txt";
std::string TraceFile = "gcvanet.tr";
std::string NetAnimFile = "buildings.xml";

static void
GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

class VanetSims
{
public:
    VanetSims ();  
    /** 
     * Creates a new building with the given params
     * 
     * \param x_min attribute
     * \param x_max attribute
     * \param y_min
     * \param y_max  
     * \param z_min
     * \param z_max
     * \param NFloors
     * \param NRoomsX
     * \param NRoomsY
     * \param Btype
     * \param WallsType
    */
//    void CreateBuilding(double x_min, double x_max, double y_min,
//        double y_max, double z_min = 0.0, double z_max = 10.0, int NFloors = 2,
//        int NRoomsX = 2, int NRoomsY = 2, Building::BuildingType_t Btype = Building::Residential,
//        Building::ExtWallsType_t WallsType = Building::ConcreteWithWindows);
    /** 
     * Creates a grid of buildings with the given params
     *  
    */
//    void CreateBuildingsGrid (uint GridWidth, double LengthX, double LengthY,
//        double DeltaX, double DeltaY, double Height, uint NRoomsX, uint NRoomsY, double NFloors,
//        double MinX = 0.0, double MinY = 0.0);
    
    void CreateNodes ();
    void CreateBuildings ();
//    void CreateBuilding();
    void SetPosition (Ptr<Node> node, Vector position);
    void CreateDevs_80211p ();
    void CreateDevs_80211p_OCB ();
    void CreateDevs_80211b ();
    void CreateDevs_80211a ();
    void InitEnergy();
    void SetRoutingAndInternet ();
    void InstallApplications ();
    void ReceivePacket (Ptr<Socket> socket);
    void Configure (int argc, char **argv);
    void clearLogFiles ();
    void SetLogComponents ();
    void TraceEnergy();
    void ExportEnergyPerNode();
    void TotalEnergy (double oldValue, double totalEnergy);
    void RemainingEnergy (double oldValue, double remainingEnergy);
    void trace_node_mobility (NodeContainer nodes);
    void ExportTraceFiles();
    void PrintAppOutput();
    void SetNetAnim();
    void installGpsr_IfChosen ();

private:
    inline std::string PrintReceivedPacket (Ptr<Socket> socket,
                                Ptr<Packet> packet);
    
public:
    uint32_t BSNodesNum; // num of Base Stations
    uint32_t STANodesNum; // num of vehicles
    uint32_t packetSize; // bytes
    double interval; // seconds
    Time interPacketInterval;
    uint32_t numOfPacks; // Number of packets to send
    uint32_t sinkNode;
    uint32_t sourceNode;
    std::string m_protocolName; // Routing protocol name
    uint32_t m_protocol; // 1: OLSR, 2: AODV, 3: DSDV, 4: DSR, 5: GPSR
    uint32_t application; // 1: Socket Udp, 2: UDP, ..
    std::string appName;   
    double startTime;
    double endTime;
    bool verbose;
    bool tracing;
    bool energyTracing;
    /*Counters and stuff*/
    double NodeTotEnergyCons[veh_num];
    uint32_t bytesTotal;
    uint32_t packetsReceived;
    uint32_t iter;
    double throughput;
    std::string mobFile; //mobility file
    
    /*Countainers*/
    NodeContainer STANodes;
    NodeContainer BSNodes;
     
    NetDeviceContainer STAdevices;
    NetDeviceContainer BSdevices;
    
    EnergySourceContainer sources;
    DeviceEnergyModelContainer deviceModels;
    
    Ipv4InterfaceContainer STAIfaceContainer;
    Ipv4InterfaceContainer BSIfaceContainer;
    
    ApplicationContainer serverApps; 
    ApplicationContainer clientApps;
    
    /*Helpers*/
    YansWifiPhyHelper wifiPhy;
    
};

VanetSims::VanetSims () :
    BSNodesNum (2),
    STANodesNum (veh_num),
    packetSize(64),
    interval (0.01),
    interPacketInterval (Seconds (interval)),
    numOfPacks (10),
    sinkNode (1),
    sourceNode(0),
    m_protocolName("NotSelectedYet"),
    m_protocol(2),
    application(4),
    appName("appName"),
    startTime (1.0),
    endTime (10),
    verbose (false),
    tracing (true),
    energyTracing (true),
    bytesTotal (0),
    packetsReceived (0),
    iter (0),
    throughput (0),
    mobFile ("mobility/topos/mob_100_22.ns2")
               
{
}

void
VanetSims::Configure (int argc, char **argv)
{
    SeedManager::SetSeed(12345);
    CommandLine cmd;
    cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
    cmd.AddValue ("numOfPacks", "number of packets generated", numOfPacks);
    cmd.AddValue ("interval", "interval (seconds) between packets", interval);
    cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
    cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
    cmd.AddValue ("energyTracing", "turn on ascii and pcap tracing", energyTracing);
    cmd.AddValue ("sinkNode", "Receiver node number", sinkNode);
    cmd.AddValue ("sourceNode", "Sender node number", sourceNode);
    cmd.AddValue ("RP", "Routing protocol", m_protocol);
    cmd.AddValue ("application", "application used", application);
    cmd.AddValue ("mobFile", "mobility file", mobFile);
    cmd.Parse (argc, argv);
}

//void 
//VanetSims::CreateBuilding(double x_min, double x_max, double y_min,
//        double y_max, double z_min = 0.0, double z_max = 10.0, int NFloors = 2,
//        int NRoomsX = 2, int NRoomsY = 2, Building::BuildingType_t Btype = Building::Residential,
//        Building::ExtWallsType_t WallsType = Building::ConcreteWithWindows)
//{
//    Ptr<Building> b = CreateObject <Building> ();
//    b->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
//    b->SetBuildingType (Btype);
//    b->SetExtWallsType (WallsType);
//    b->SetNFloors (NFloors);
//    b->SetNRoomsX (NRoomsX);
//    b->SetNRoomsY (NRoomsY);
//}

//void 
//VanetSims::CreateBuildingsGrid (uint GridWidth, double LengthX, double LengthY,
//        double DeltaX, double DeltaY, double Height, uint NRoomsX, uint NRoomsY, double NFloors,
//        double MinX = 0.0, double MinY = 0.0)
//{
//    Ptr<GridBuildingAllocator>  gridBuildingAllocator;
//    gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
//    gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (GridWidth));
//    gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (LengthX));
//    gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (LengthY));
//    gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (DeltaX));
//    gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (DeltaY));
//    gridBuildingAllocator->SetAttribute ("Height", DoubleValue (Height));
//    gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (NRoomsX));
//    gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (NRoomsY));
//    gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (NFloors));
//    gridBuildingAllocator->SetAttribute ("MinX", DoubleValue (MinX));
//    gridBuildingAllocator->SetAttribute ("MinY", DoubleValue (MinY));
//    gridBuildingAllocator->Create (6);
//}

void
VanetSims::CreateBuildings()
{
    double x_min = 150.0;
    double x_max = 250.0;
    double y_min = 100.0;
    double y_max = 200.0;
    double z_min = 1.0;
    double z_max = 30.0;
    Ptr<Building> b = CreateObject <Building> ();
    b->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
    b->SetBuildingType (Building::Residential);
    b->SetExtWallsType (Building::ConcreteWithWindows);
    b->SetNFloors (1);
    b->SetNRoomsX (1);
    b->SetNRoomsY (1);
    
    BuildingsHelper::Install (STANodes);
    BuildingsHelper::Install (BSNodes);
    BuildingsHelper::MakeMobilityModelConsistent ();
    
//    BuildingList::GetNBuildings();    
}
void
VanetSims::CreateNodes()
{
    STANodes.Create (2);
    BSNodes.Create (4);
    
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (STANodes);
    mobility.Install (BSNodes);
    
    SetPosition (STANodes.Get(0), Vector (100.0, 150.0, 1.0));
    SetPosition (STANodes.Get(1), Vector (100.0, 10.0, 1.0)); 
//    SetPosition (STANodes.Get(1), Vector (300.0, 150.0, 1.0)); 
    
    SetPosition (BSNodes.Get(0), Vector (150.0, 100.0, 1.0));
    SetPosition (BSNodes.Get(1), Vector (150.0, 200.0, 1.0)); 
    SetPosition (BSNodes.Get(2), Vector (250.0, 100.0, 1.0));
    SetPosition (BSNodes.Get(3), Vector (250.0, 200.0, 1.0)); 
}

void
VanetSims::CreateDevs_80211p ()
{   
    std::string phyMode ("OfdmRate6MbpsBW10MHz");
    
    // disable fragmentation for frames below 2200 bytes
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
    // turn off RTS/CTS for frames below 2200 bytes
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
    // Fix non-unicast data rate to be the same as that of unicast
    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
    
    // Channel - Physical Layer
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
//    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    
    Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
    wifiChannel.AddPropagationLoss ("ns3::HybridBuildingsPropagationLossModel",
                                    "CitySize", StringValue("Small"),
                                    "ShadowSigmaOutdoor", DoubleValue (10.0),
                                    "ShadowSigmaExtWalls", DoubleValue (10.0),
                                    "InternalWallLoss", DoubleValue (10.0),
                                    "Environment", StringValue("Urban"));
    
    // WifiPhysical
    wifiPhy =  YansWifiPhyHelper::Default ();  
    wifiPhy.SetChannel (wifiChannel.Create ());
    
    /* 300m */
    wifiPhy.Set ("RxGain", DoubleValue (-10) ); // set it to zero; otherwise, gain will be added  
    wifiPhy.Set ("TxPowerStart", DoubleValue(15));
    wifiPhy.Set ("TxPowerEnd", DoubleValue(15));
    
    
     /*WIFI_80211p ADHOC*/
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
    wifiMac.SetType ("ns3::AdhocWifiMac");

    WifiHelper wifi = WifiHelper::Default ();
    wifi.SetStandard (WIFI_PHY_STANDARD_80211_10MHZ);
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue (phyMode),
                                  "ControlMode",StringValue (phyMode));
    /*Install WIFI*/
    STAdevices = wifi.Install (wifiPhy, wifiMac, STANodes);
//    BSdevices = wifi.Install (wifiPhy, wifiMac, BSNodes);
    if (verbose) wifi.EnableLogComponents (); //Turn on Wifi 802.11p logging
}    

void
VanetSims::CreateDevs_80211p_OCB ()
{
     std::string phyMode ("OfdmRate6MbpsBW10MHz");
            
    // Channel - Physical Layer
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

    // WifiPhysical
    wifiPhy =  YansWifiPhyHelper::Default ();  
    wifiPhy.Set ("RxGain", DoubleValue (-10) ); // set it to zero; otherwise, gain will be added  
    wifiPhy.SetChannel (wifiChannel.Create ());
    
    /*WIFI_80211p OCB*/
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
    
    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
    NetDeviceContainer STAdevices = wifi80211p.Install (wifiPhy, wifi80211pMac, STANodes);
//    NetDeviceContainer BSdevices = wifi80211p.Install (wifiPhy, wifi80211pMac, BSNodes);
}

void
VanetSims::CreateDevs_80211b ()
{
    std::string phyMode ("DsssRate11Mbps"); //gia 802.11b

    /*Channel - Physical Layer*/
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

    /*WifiPhysical*/
    wifiPhy =  YansWifiPhyHelper::Default ();  
    wifiPhy.Set ("RxGain", DoubleValue (-10) ); // set it to zero; otherwise, gain will be added  
    wifiPhy.SetChannel (wifiChannel.Create ());

    /*ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);*/
    
    /*WIFI_PHY_STANDARD_80211b*/
    WifiHelper wifi;
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
    wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue (phyMode),
                                  "ControlMode",StringValue (phyMode));
    /*Set network to Ad Hoc mode*/
    wifiMac.SetType ("ns3::AdhocWifiMac");
    
    /*Install WIFI*/
    STAdevices = wifi.Install (wifiPhy, wifiMac, STANodes);
//    BSdevices = wifi.Install (wifiPhy, wifiMac, BSNodes);
}

void
VanetSims::CreateDevs_80211a ()
{
    //    Not Tested Yet !!!!!
    
    // Channel - Physical Layer
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

    // WifiPhysical
    wifiPhy =  YansWifiPhyHelper::Default ();  
    wifiPhy.Set ("RxGain", DoubleValue (-10) ); // set it to zero; otherwise, gain will be added  
    wifiPhy.SetChannel (wifiChannel.Create ());
    
    /*WIFI_PHY_STANDARD_80211a*/
     WifiHelper wifi = WifiHelper::Default ();
     wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
     NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
     YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

    /*Set network to Ad Hoc mode*/
    wifiMac.SetType ("ns3::AdhocWifiMac");

    /*Install WIFI*/
    NetDeviceContainer STAdevices = wifi.Install (wifiPhy, wifiMac, STANodes);
//    NetDeviceContainer BSdevices = wifi.Install (wifiPhy, wifiMac, BSNodes);
}

void
VanetSims::InitEnergy()
{
    /** Energy Model **/
    /* energy source */
    BasicEnergySourceHelper basicSourceHelper;
    // configure energy source
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.1));
    // install source
    sources.Add(basicSourceHelper.Install (STANodes));
    /* device energy model */
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
    // install device model
    deviceModels = radioEnergyHelper.Install (STAdevices, sources);
}

void
VanetSims::SetRoutingAndInternet()
{
    AodvHelper aodv;
    OlsrHelper olsr;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;
    GpsrHelper gpsr;

    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    // list.Add (staticRouting, 0);
    
    switch (m_protocol)
    {
      case 1:
        list.Add (olsr, 1);
        m_protocolName = "OLSR";
        break;
      case 2:
        list.Add (aodv, 1);
        m_protocolName = "AODV";
        break;
      case 3:
        list.Add (dsdv, 1);
        m_protocolName = "DSDV";
        break;
      case 4:
        m_protocolName = "DSR";
        break;
       case 5:
        list.Add (gpsr, 1);
        m_protocolName = "GPSR";
        break;
      default:
        NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

    InternetStackHelper internet;

    if (m_protocol < 4)
    {
        internet.SetRoutingHelper (list);
        internet.Install (STANodes);
//        internet.Install (BSNodes);
    }
    else if (m_protocol == 4)
    {
        internet.Install (STANodes);
//        internet.Install (BSNodes);
        dsrMain.Install (dsr, STANodes);
//        dsrMain.Install (dsr, BSNodes);
    }
    else if (m_protocol == 5)
    {
        internet.SetRoutingHelper (gpsr);
        internet.Install (STANodes);
//        internet.Install (BSNodes);        
    }

    Ipv4AddressHelper ipv4;
//    NS_LOG_INFO ("Assign IP Addresses.");
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    STAIfaceContainer = ipv4.Assign (STAdevices);
//    BSIfaceContainer = ipv4.Assign (BSdevices);
}

void
VanetSims::InstallApplications()
{   
    switch (application)
    {
        case 1:
        {
            ;
        }
        break;
        case 2:
        {
            ;
        }
        break;
        case 3:
        {
            ;
        }
        break;
        case 4:
        {
            appName = "Udp Unicast Application";
            
            UdpServerHelper Server (9);         

            serverApps.Add (Server.Install (STANodes.Get (1)));
            
            serverApps.Start (Seconds (startTime));
            serverApps.Stop (Seconds (endTime));
            
            serverApps.Get(0)->SetStartTime(Seconds(1));
            serverApps.Get(0)->SetStopTime(Seconds(10));          

            UdpClientHelper Client0 (STAIfaceContainer.GetAddress (1), 9);
            
            Client0.SetAttribute ("MaxPackets", UintegerValue (numOfPacks));
            Client0.SetAttribute ("Interval", TimeValue (Seconds (interval)));
            Client0.SetAttribute ("PacketSize", UintegerValue (packetSize));                       
            
            clientApps.Add(Client0.Install (STANodes.Get (0)));           
              
            clientApps.Start (Seconds (startTime+1.0));
            clientApps.Stop (Seconds (endTime));

            clientApps.Get(0)->SetStartTime(Seconds(2));
            clientApps.Get(0)->SetStopTime(Seconds(endTime));

        }
        break;
        case 5:
        {
            appName = "Tcp Application";
            // uint16_t port = 50000;
            // Address apLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
            // PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", apLocalAddress);
            // sink1App = packetSinkHelper.Install (wifiNodes.Get (0));
            //
            // sink1App.Start (Seconds (0.0));
            // sink1App.Stop (Seconds (t));
            //
            // OnOffHelper onoff ("ns3::TcpSocketFactory",Ipv4Address::GetAny ());
            // onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable(30)));//in seconds
            // onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable(0)));
            // onoff.SetAttribute ("PacketSize", UintegerValue (1500-30));//1024
            // onoff.SetAttribute ("DataRate", DataRateValue (100000000));//51200
            // ApplicationContainer apps;
            //
            // AddressValue remoteAddress (InetSocketAddress (wifiNodesInterfaces.GetAddress(0), port));
            // onoff.SetAttribute ("Remote", remoteAddress);
            // apps.Add (onoff.Install (wifiApNode.Get (0)));
            // apps.Start (Seconds (0.0));
            // apps.Stop (Seconds (t));
        }
        break;
    }
}

inline std::string
VanetSims::PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet)
{
  std::ofstream logs;
  logs.open ((outdir + log_4).c_str(), std::ios::app);

  SocketAddressTag tag;
  bool found;
  found = packet->PeekPacketTag (tag);
  std::ostringstream oss;

  logs << Simulator::Now ().GetSeconds () << " " << "Node: " <<
          socket->GetNode ()->GetId ();
  //oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (found)
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (tag.GetAddress ());
      logs << " received 1 packet with uid: " << packet->GetUid() << " size: "
              << packet->GetSize() << " bytes from " 
              << addr.GetIpv4 () << " port: " << addr.GetPort () << "\n";
      //oss << " received one packet from " << addr.GetIpv4 () << " port: " << addr.GetPort ();
    }
  else
    {
      //oss << " received one packet!";
      logs << " \nreceived one packet!!!\n";
    }
  logs.close();
  return oss.str ();
}

void
VanetSims::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      // bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      PrintReceivedPacket (socket, packet);
      //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet));
    }
}

void 
VanetSims::trace_node_mobility (NodeContainer nodes) 
{ 
  std::ofstream logs;
  logs.open ((outdir + log_2).c_str(), std::ios::app);

  // iterate our nodes and print their position.
  for (NodeContainer::Iterator j = nodes.Begin ();
       j != nodes.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Vector pos = position->GetPosition ();
      logs << "Node: " << object->GetId() << " x= " << pos.x << " y= " << pos.y << " z= " << pos.z << "\n";
    }
  logs.close();
}

//Set position of the nodes
void
VanetSims::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

void
VanetSims::TraceEnergy()
{
    // Energy tracing
    for (uint32_t i = 0; i < STANodesNum; ++i) {
      // Energy source
      Ptr<BasicEnergySource> basicSourcePtr = 
              DynamicCast<BasicEnergySource> (sources.Get (i));
      // basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&VanetSims::RemainingEnergy, this));
      // device energy model
      Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->
              FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
      NS_ASSERT (basicRadioModelPtr != NULL);
      basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption",
              MakeCallback (&VanetSims::TotalEnergy, this));
    }
    
    Simulator::Schedule (Seconds (startTime), &VanetSims::trace_node_mobility, this, STANodes);
    Simulator::Schedule (Seconds (endTime - 1), &VanetSims::ExportEnergyPerNode,this);
}

void
VanetSims::RemainingEnergy (double oldValue, double remainingEnergy)
{
//  NS_LOG_UNCOND ("s Current remaining energy = " << remainingEnergy << " J");
}

// Trace function for total energy consumption at node.
void
VanetSims::TotalEnergy (double oldValue, double totalEnergy)
{
  int index_node = iter%STANodesNum;
  NodeTotEnergyCons[index_node] = totalEnergy;
  iter ++;
}

void
VanetSims::ExportEnergyPerNode ()
{
  std::ofstream logs;
  std::string rp = "rp_1_";
   switch (m_protocol)
    {
      case 1:
        rp = "rp_1_";
        break;
      case 2:
        rp = "rp_2_";
        break;
      case 3:
        rp = "rp_3_";
        break;
      case 4:
        rp = "rp_4_";
        break;
       case 5:
        rp = "rp_5_";
        break;
      default:
        NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }
   
  logs.open ((outdir + rp + log_1).c_str(),std::ios::app);
  for (uint32_t i = 0; i < STANodesNum; ++i) {
    logs << i << " " << NodeTotEnergyCons[i] << "\n";
  }
  logs.close();
}

void
VanetSims::clearLogFiles ()
{
    // Clear files for logging..
    std::ofstream logf1;
    logf1.open ((outdir + log_1).c_str(),std::ios::trunc);
    logf1.close();
    
    std::ofstream logf2;
    logf2.open ((outdir + log_2).c_str(),std::ios::trunc);
    logf2.close();
    
    std::ofstream logf3;
    logf3.open ((outdir + log_3).c_str(),std::ios::trunc);
    logf3.close();

    std::ofstream logf4;
    logf4.open ((outdir + log_4).c_str(),std::ios::trunc);
    logf4.close();
}

void
VanetSims::ExportTraceFiles()
{
    AsciiTraceHelper ascii;
    wifiPhy.EnableAsciiAll (ascii.CreateFileStream ((outdir + TraceFile).c_str()));  
    // Trace nodes mobility
     MobilityHelper::EnableAsciiAll (ascii.CreateFileStream ("mobility-trace-example.mob"));
}

void
VanetSims::PrintAppOutput()
{
    if (application == 1)
    {
        // Socket Broadcast UDP
        std::cout << "App: " << appName << "\n";
        std::cout << "Total packets sent: " << numOfPacks * STANodesNum << " \n";
        std::cout << "Total packets received: " << packetsReceived << "\n";
        float pdr = (float) packetsReceived / (float) (numOfPacks * STANodesNum); 
        std::cout <<"PDR:  " << pdr * 100 << "%\n";
        std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";     
    }
    else if (application == 2)
    {
        // Socket Unicast UDP
        std::cout << "App: " << appName << "\n";
        std::cout << "Total packets sent: " << numOfPacks << " \n";
        std::cout << "Total packets received: " << packetsReceived << "\n";
        float pdr = (float) packetsReceived / (float) (numOfPacks); 
        std::cout <<"PDR:  " << pdr * 100 << "%\n";
        std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";    
    }
    else if (application == 3)
    {
        // Broadcast UDP App
        uint32_t nApplications = serverApps.GetN ();
        for (uint32_t i = 0; i < nApplications; ++i)
          {
            Ptr<Application> app = serverApps.Get (i);
            
            uint32_t totalPacketsThrough = DynamicCast<UdpServer>(app)->GetReceived ();
            throughput = totalPacketsThrough*1500*8/(endTime*1000000.0);
            float pdr = (float) totalPacketsThrough / (float) numOfPacks ;
            std::cout << "App: " << appName << "\n";
            std::cout <<"App for node: " << app->GetNode() << "\n-----------\n";
            std::cout << "Total packets sent: " << numOfPacks << " \n";
            std::cout <<"totalPacketsThrough  " << totalPacketsThrough << "\n";
            std::cout <<"Throughput  " << throughput << "\n";
            std::cout <<"PDR:  " << pdr * 100 << "%\n";
            std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";
          }
    }
    else if (application == 4)
    {
        // // Unicast UDP App
        // uint32_t totalPacketsThrough = DynamicCast<UdpServer>(serverApps.Get (0))->GetReceived ();
        // throughput = totalPacketsThrough*1500*8/(endTime*1000000.0);
        // float pdr = (float) totalPacketsThrough / (float) numOfPacks ;
        // std::cout << "App: " << appName << "\n";
        // std::cout <<"\nApp for node: " << serverApps.Get (0)->GetNode() << "\n-----------\n";
        // std::cout << "Total packets sent: " << numOfPacks << " \n";
        // std::cout <<"totalPacketsThrough  " << totalPacketsThrough << "\n";
        // std::cout <<"Throughput  " << throughput << "\n";
        // std::cout <<"PDR:  " << pdr * 100 << "%\n";
        // std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";
        
        // totalPacketsThrough = DynamicCast<UdpServer>(serverApps.Get (1))->GetReceived ();
        // throughput = totalPacketsThrough*1500*8/(endTime*1000000.0);
        // pdr = (float) totalPacketsThrough / (float) numOfPacks ;
        // std::cout << "App: " << appName << "\n";
        // std::cout <<"\nApp for node: " << serverApps.Get (1)->GetNode() << "\n-----------\n";
        // std::cout << "Total packets sent: " << numOfPacks << " \n";
        // std::cout <<"totalPacketsThrough  " << totalPacketsThrough << "\n";
        // std::cout <<"Throughput  " << throughput << "\n";
        // std::cout <<"PDR:  " << pdr * 100 << "%\n";
        // std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";
        
        // totalPacketsThrough = DynamicCast<UdpServer>(serverApps.Get (2))->GetReceived ();
        // throughput = totalPacketsThrough*1500*8/(endTime*1000000.0);
        // pdr = (float) totalPacketsThrough / (float) numOfPacks ;
        // std::cout << "App: " << appName << "\n";
        // std::cout <<"\nApp for node: " << serverApps.Get (2)->GetNode() << "\n-----------\n";
        // std::cout << "Total packets sent: " << numOfPacks << " \n";
        // std::cout <<"totalPacketsThrough  " << totalPacketsThrough << "\n";
        // std::cout <<"Throughput  " << throughput << "\n";
        // std::cout <<"PDR:  " << pdr * 100 << "%\n";
        // std::cout <<"PLR:  " << (1.0 - pdr) * 100 << "%\n";
    }
}

void
VanetSims::SetNetAnim()
{ 
//    AnimationInterface::SetNodeDescription (BSNodes, "BS"); // Optional
//    AnimationInterface::SetNodeColor (BSNodes, 0, 255, 0);
//    
//    AnimationInterface::SetNodeDescription (STANodes.Get(26), "DstVeh1"); // Optional
//    AnimationInterface::SetNodeColor (STANodes.Get(26), 0, 255, 0);
//    
//    AnimationInterface::SetNodeDescription (STANodes.Get(42), "DstVeh2"); // Optional
//    AnimationInterface::SetNodeColor (STANodes.Get(42), 0, 255, 0);
    
    AnimationInterface anim ((outdir + NetAnimFile).c_str()); // Mandatory
    anim.EnablePacketMetadata (true); // Optional
//    anim.EnableIpv4RouteTracking ("routingtable-veh.xml", Seconds(20), Seconds(40), Seconds(0.25)); //Optional
}

void
VanetSims::SetLogComponents ()
{
    LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
    LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
//    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_DEBUG);
}

void
VanetSims::installGpsr_IfChosen()
{
    if (m_protocol == 5)
    {
        GpsrHelper gpsr;
        gpsr.Install ();
    }
}

int 
main (int argc, char *argv[])
{
    VanetSims vanet;
    
    vanet.clearLogFiles();
    vanet.SetLogComponents();
    
    /* Get Input params */
    vanet.Configure(argc,argv);
    
    /* Nodes and mobility */
    vanet.CreateNodes ();
    vanet.CreateBuildings ();
    
    /*  Channel and devices */
    vanet.CreateDevs_80211p ();
    
    /*  Routing and Internet Stack */
    vanet.SetRoutingAndInternet ();
    
    /*  Applications */
    vanet.InstallApplications ();
            
    vanet.installGpsr_IfChosen();
    vanet.SetNetAnim();       
    
    Simulator::Stop (Seconds (vanet.endTime + 2.0));
    Simulator::Run ();   
    Simulator::Destroy ();
    
    vanet.PrintAppOutput();
    
    return 0;
}
