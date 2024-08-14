using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;
using System.Timers; // To define fixed time intervalls when twin is updated or data is from twin is read
// we are using netcore version 8 on ubuntu, check the config before running to ensure the NetCore version is 8.0
namespace CSClient
{
    class ForwardHelper
    {   
        // Robot control station ip (localhost if on this machine)
        string CONTROL_IP = "127.0.0.1";
        int CONTROL_PORT = 8080;
        string PROVIDER_IP = "192.168.10.1";
        int PROVIDER_PORT = 1234;
        int SERVER_PORT = 45455;
        string CONSUMER_IP = "192.168.10.15"; 
        // Simulation will connect to this
        TcpListener m_tcpServer;
        TcpClient m_tcpClientConsumer;
        NetworkStream m_consumerStream;
        // Connects to the robot control as a client
        TcpClient m_tcpClientControl;
        NetworkStream m_controlStream;
        //
        TcpClient m_tcpClientSim;
        NetworkStream m_simStream;
        

        public void start_as_provider(){
            
            // Establish a connection with the robot control programm
            Console.WriteLine("[INFO] Trying to connect to robot control");
            m_tcpClientControl = new TcpClient(CONTROL_IP, CONTROL_PORT);
            Console.WriteLine("[INFO] Connected to the robot control");
            Console.WriteLine("[INFO] Initializing server");
            // Establish a connection with the consumer side
            m_tcpServer = new TcpListener(PROVIDER_PORT);
            m_tcpServer.Start();
            Console.WriteLine("[INFO] Started the server");
            m_tcpClientConsumer = m_tcpServer.AcceptTcpClient();
            m_consumerStream = m_tcpClientConsumer.GetStream();
            Console.WriteLine("[INFO] Got the connection of the consumer client");
        }

        public void start_as_consumer(){
            // First connecto to the provider instance of this class
            Console.WriteLine("[INFO] Trying to connec to the provider instance");
            m_tcpClientConsumer = new TcpClient(PROVIDER_IP, PROVIDER_PORT);
            Console.WriteLine("[INFO] Connected to provider instance");
            Console.WriteLine("[INFO] Waiting for simulation to connect");
            m_tcpServer = new TcpListener(IPAddress.Any, SERVER_PORT);
            m_tcpServer.Start();
            m_tcpClientSim = m_tcpServer.AcceptTcpClient();
            m_simStream = m_tcpClientSim.GetStream();
            Console.WriteLine("[INFO] Got the connection of the simulation client");

        }

    }
}
