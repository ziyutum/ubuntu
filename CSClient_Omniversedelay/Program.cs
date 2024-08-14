﻿
using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Text.Json; // For deserialization of the dtdl description of the twin instances //  and for deserialization of operational data to objects where we don't have a defined type
using System.Timers; // To define fixed time intervalls when twin is updated or data is from twin is read

// Azure, Azure Digital Twins realted imports
// Note: The packages must be added to the project first, see
//       https://docs.microsoft.com/de-de/azure/digital-twins/tutorial-code
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;

namespace CSClient
{
    class Program
    {
        // Variables to time processes, initially for debugging
        private static Timer sysTimer;
        private static int iTimerCounter = 0; // Counts how often timer elpased, to stop process at some point
        private static bool bSendFromRobotTcpStream = false; // Set to true when timer elapsed; send to false after data was send

        // The Azure Digital Twins Client
        static DigitalTwinsClient m_azureClient;

        // Members for the consumer connection
        static int m_serverPort = 32323; // 32323
        static TcpListener m_tcpServer;
        static TcpClient m_tcpClientConsumer;
        static NetworkStream m_consumerStream;
        static Timer m_timerAcceptClient;
        static bool m_bConsumerIsConnected;
        
        // Members to configure the downstream access
        static Timer m_downstreamTimer = new System.Timers.Timer(100); // Max length in ms
        static Timer m_maxTimeDownstream = new System.Timers.Timer(60000*5000); // after that csv is created
        static List<string> m_downstreamTwins = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", 
                                    "JointPosition5", "JointPosition6", "JointPosition7", "GripperData_Width", "GripperData_Speed",
                                    "Bottle_LocationX", "Bottle_LocationY"};
        
        // Vars for debugging
        static List<string> m_dataAzdtDownload = new List<string>();

        static int m_numDataRequested  = 0;

        // ENTRY POINT 
        static async Task Main(string[] args)
        {
            Console.WriteLine("Started the Client-App.\nBeginning with authentication ...\n");
            // Instace URL found in the Azure Portal
            string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";
            //  https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net 
            //Authenticate with the Azure service
            var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
            DigitalTwinsClient client = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
            m_azureClient = client;
            Console.WriteLine("Service client succsessfully created.");
            
            //------ Setup and Initalization Functions --------
            initialize_consumer_connection(); // Consumer is Isaac Sim  // Currently commented out because no Omniverse Isaac Sim
            //-------------------------------------------------
            
            //----- Specify conditions for communication start ----- // Commented out below because no connection to Omiverse necessary
            while( /*!m_bConnectedToDevice ||*/ !m_bConsumerIsConnected ) { 
            //    // Console.WriteLine("Waiting until conditions are satisfied");
               await Task.Delay(1000);}
               // N.B.: We use a slow poll loop instead of a busy-wait loop which is possible 
               //       because the main method is specified as a async task.
            //------------------------------------------------------ 

            //------ Communication Functions -----------------
            
            downstream_simulation_forwarding();
            
            //-------------------------------------------------
            
            Console.WriteLine("Press Enter to end program ...");
            Console.ReadLine();
        }
        
        // Establishes connection with a tcp client, for instance the panda robot
        
        static void initialize_consumer_connection()
        {
            m_tcpServer = new TcpListener(IPAddress.Any, m_serverPort);
            m_tcpServer.Start();
            m_timerAcceptClient = new System.Timers.Timer(5000);
            m_timerAcceptClient.Elapsed += call_accept;
            m_timerAcceptClient.Start();

        }

        static void call_accept(Object source, ElapsedEventArgs e){
            if(!m_bConsumerIsConnected)
                try_accept_consumer();
        }

        static void try_accept_consumer()
        {
            try
            {
                m_tcpClientConsumer = m_tcpServer.AcceptTcpClient();
                m_consumerStream = m_tcpClientConsumer.GetStream();
                m_bConsumerIsConnected = true;
                Console.WriteLine("[INFO] Consumer is connected");
                //if(m_bConnectedToDevice && m_bConsumerIsConnected)
                //    process_and_forward_fci_data();
            }
            catch
            {
                Console.WriteLine("[WARN] Error in consumer connection establishment.");
            }
        }           

        /* 
         *  Access the data from the Azure Digital Twins instance, create an array of the joint values
         *  and forward it to the simulation.
         */
        static void downstream_simulation_forwarding()
        {
            // Initialize the downstream access timers
            m_downstreamTimer.Elapsed += call_downstream_access;   // Function to handle timer event
            m_downstreamTimer.Enabled = true;                      // Start the timer 
            m_maxTimeDownstream.Elapsed += stop_downstream_access; // Clean up after timeout
            m_maxTimeDownstream.Enabled = true;                    // Start max connection timer
            Console.WriteLine("[INFO] Started accessing the Azure downstream");

            //@brief Called when the timer for downstream access elapsas and handels
            //        the actual call to the methods that accesses Azure.
            void call_downstream_access(Object source, ElapsedEventArgs e){
                //Console.WriteLine("[INFO] Calling Downstream Access");
                donwstream_access();
            }

            //@brief Implements our azure downstream, i.e. it gets the relevant data from the digitial
            //        twin instance and sends it to further processes, e.g. omniverse.
            void donwstream_access()
            {
                int numDataRequested = m_numDataRequested;
                m_numDataRequested++;


                // Record timestamp before data download
                string startTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");

                string component_value_result = String.Empty;          
                BasicDigitalTwin twin_value; 
                string buffer = "[";
                        
                foreach (var joint_twin in m_downstreamTwins)
                {
                    Response<BasicDigitalTwin> getTwin_Response = m_azureClient.GetDigitalTwin<BasicDigitalTwin>(joint_twin);
                    twin_value = getTwin_Response.Value;
                    
                    component_value_result = twin_value.Contents["value"].ToString();  
                    buffer += component_value_result;
                    buffer += ",";
                }
                buffer = buffer.Remove(buffer.Length -1); // remove last comma
                buffer += "]";
                 // Record timestamp after data download
                string endTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");

                // Addon: Exporting mechanism for received data
                m_dataAzdtDownload.Add(buffer.Substring(1, buffer.Length -2));

                byte[] byte_buffer = new byte[100]; 
                byte_buffer = Encoding.Default.GetBytes(buffer);  // convert string to an byte array
                m_consumerStream.Write(byte_buffer, 0, byte_buffer.Length);     //sending the message // Commented out because Currently no Connection to Isaac-SIM
                Console.WriteLine("[INFO] Downloaded and Forwarded data " + numDataRequested + " times");
                AppendTimestampsToCsv(startTimestamp, endTimestamp);
            }
            void AppendTimestampsToCsv(string start, string end)
            {
                string csvFilePath = "data_export/timestamps_and_delays.csv";
                
                // Parse timestamps to DateTime
                DateTime startDateTime = DateTime.Parse(start);
                DateTime endDateTime = DateTime.Parse(end);

                // Calculate the time difference in milliseconds
                double delay = (endDateTime - startDateTime).TotalMilliseconds;

                // Prepare the CSV line with timestamps and delay
                var csvLine = $"{start},{end},{delay}";
                
                using (StreamWriter writer = new StreamWriter(csvFilePath, true))
                {
                    writer.WriteLine(csvLine);
                }
            }

            // Clean up all timers and close the connections
            void stop_downstream_access(Object source, ElapsedEventArgs e){
                Console.WriteLine("[INFO] Stopping downstream access");
                m_downstreamTimer.Stop();
                m_downstreamTimer.Dispose();
                m_maxTimeDownstream.Stop();
                m_maxTimeDownstream.Dispose();
                m_consumerStream.Close();
                m_tcpClientConsumer.Close();
                Console.WriteLine("[INFO] Closed all streams");

                // Addon: Exporting mechanism for received data
                string dataFilePath = "data_export/valid_input.csv";
                using (StreamWriter writer = new StreamWriter(dataFilePath))
                {
                    writer.WriteLine(string.Join(",",m_downstreamTwins));   
                    foreach (string str in m_dataAzdtDownload)
                    {
                        writer.WriteLine(str);
                    }
                }
                Console.WriteLine($"[SUCCESS] Exported recorded data to {dataFilePath} (Datapoints: {m_dataAzdtDownload.Count})");



                Console.WriteLine("[END] Finished downstream access");
            }
        }
    }
}