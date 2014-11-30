/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/gpsr-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/udp-server.h"
#include "ns3/udp-client.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "src/stats/model/gnuplot.h"
#include "ns3/netanim-module.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>

using namespace ns3;

/* global vars*/
std::string outdir = "logs/";
std::string TraceFile = "gpsr-test1.tr";
std::string ns2mobfile = "mobility/gpsr-test1.mob.ns2";
        

Gnuplot2dDataset Pdr("PDR %)");
Gnuplot2dDataset RoutingProtocol("RoutingProtocol)");



class GpsrExample
{
public:
  GpsrExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
  void ParseTracefile ();

private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;
  /// Width of the Node Grid
  uint32_t gridWidth;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  //\}

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  //\}

private:
  void CreateNodes ();
  void CreateNode (double posX, double posY, double dstX, double dstY, double vel);
  void CreateRoad (uint32_t vehNum, double StposX, double StposY, double distance, double vel, bool rightDir);
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
};

//-----------------------------------------------------------------------------
GpsrExample::GpsrExample () :
  // Number of Nodes
  size (0),
  // Grid Width
  gridWidth(2),
  // Distance between nodes
  step (100), //TODO Distance changed to the limit between nodes: test to see if there are transmitions
  // Simulation time
  totalTime (15),
  // Generate capture files for each node
  pcap (false)

{
}

bool
GpsrExample::Configure (int argc, char **argv)
{
  // Enable GPSR logs by default. Comment this if too noisy
  // LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_ALL);

  SeedManager::SetSeed(12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);

  cmd.Parse (argc, argv);
  return true;
}

void
GpsrExample::ParseTracefile ()
{
    /* Parse Tracefile and remove null characters*/
    std::ifstream in((outdir + TraceFile).c_str());
//    in.close();
    std::stringstream out;
    std::stringstream temp;
    std::ofstream ntrace;
    ntrace.open ("test1.txt", std::ios::trunc);
   
  
    out << in.rdbuf();
    
//    int i = 0;
    
    
//    ntrace << out.str();
    while (out)
    {        
        char c;
        c = out.get();
//        temp << c;
        if ((isprint(c)))
        {
            temp << c;
        }
        else
        {
            ntrace << "";           
        }
        
    }
    ntrace << temp.str();
    
    ntrace.close();
}

void
GpsrExample::Run ()
{
   
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  GpsrHelper gpsr;
  gpsr.Install ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.InstallAll();
  
  Simulator::Stop (Seconds (totalTime));  
  Simulator::Run ();
   
  //ParseTracefile ();
  Simulator::Destroy ();
  
  flowMonitor->SerializeToXmlFile("NameOfFile.xml", true, true);
}

void
GpsrExample::Report (std::ostream &)
{
}

void
GpsrExample::CreateNode (double posX, double posY, double dstX, double dstY, double vel)
{
    std::ofstream os;
    os.open ((ns2mobfile).c_str(), std::ios::app);
    
    os << "$node_("<< size << ") set X_ " << posX << "\n"
       << "$node_("<< size << ") set Y_ " << posY << "\n"
       << "$node_("<< size << ") set Z_ 0\n"
       << "$ns_ at 1.0 \"$node_("<< size << ") setdest " 
       << dstX << " "<< dstY << " " << vel << "\"\n\n";
    size ++;

}

void
GpsrExample::CreateRoad (uint32_t vehNum, double StposX, double StposY, double distance, double vel, bool rightDir)
{
    int32_t dest;
    double dist = 0.0;
    if (rightDir) {
        dest = 10000;
    }
    else {
        dest = -10000;
    }
    
    for (uint32_t i = 0; i < vehNum; ++i)
    {
        CreateNode (StposX + dist, StposY, StposX + dest, StposY, vel);
        dist += distance;
    }
}

void
GpsrExample::CreateNodes ()
{
    std::ofstream mob;
    mob.open (ns2mobfile.c_str(),std::ios::trunc);
    mob.close();
  //std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
    
    CreateRoad(25, 100.0, 100.0 , 100.0, 28.1, true);
    CreateRoad(25, 150.0, 105.0 , 100.0, 22.1, true);
//    CreateRoad(25, 100.0, 110.0 , 100.0, 16.1, true);
    
    CreateRoad(25, 200.0, 75.0 , 100.0, 28.1, false);
    CreateRoad(25, 150.0, 80.0 , 100.0, 22.1, false);
//    CreateRoad(25, 200.0, 85.0 , 100.0, 16.1, false);
    
    
//   CreateNode(1040.0, 100.0, 10000.0, 100.0, 40.0);

  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
     {
       std::ostringstream os;
       os << "node-" << i;
       Names::Add (os.str (), nodes.Get (i));
     }
  
  std::string traceFile = ns2mobfile;
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile); 
  ns2.Install(nodes.Begin(), nodes.End());

}

void
GpsrExample::CreateDevices ()
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
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

    // WifiPhysical
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();      
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
    
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {     
//      wifiPhy.EnablePcapAll (std::string ("gpsr"));
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ((outdir + TraceFile).c_str()));
    }
}

void
GpsrExample::InstallInternetStack ()
{
//    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_FUNCTION);
  GpsrHelper gpsr;
  // you can configure GPSR attributes here using gpsr.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (gpsr);
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.0.0");
  interfaces = address.Assign (devices);
}

void
GpsrExample::InstallApplications ()
{
//  LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
//  LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
//  LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_ALL);
  uint16_t receiver = 2;
  uint16_t sender = 0;
  
  uint16_t port = 9;  // well-known echo port number
  uint32_t packetSize = 256; // size of the packets being transmitted
  uint32_t maxPacketCount = 20; // number of packets to transmit
  Time interPacketInterval = Seconds (0.01); // interval between packet transmissions

  // Set-up a server Application on the bottom-right node of the grid
  UdpServerHelper server1 (port);
  uint16_t server1Position = receiver; //bottom right
  ApplicationContainer apps = server1.Install (nodes.Get(server1Position));
  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (totalTime-0.1));

  // Set-up a client Application, connected to 'server', to be run on the top-left node of the grid
  UdpClientHelper client (interfaces.GetAddress (server1Position), port);
  client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client.SetAttribute ("PacketSize", UintegerValue (packetSize));
  uint16_t clientPosition = sender; //top left
  apps = client.Install (nodes.Get (clientPosition));
  apps.Start (Seconds (2.0));
  apps.Stop (Seconds (totalTime-0.1));

}

int main (int argc, char **argv)
{
  GpsrExample test;
  if (! test.Configure(argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");


  test.Run ();
  test.Report (std::cout);
  return 0;
}