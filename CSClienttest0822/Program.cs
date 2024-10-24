﻿
using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Text.Json; // For deserialization of the dtdl description of the twin instances
 //  and for deserialization of operational data to objects where we don't have a defined type
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

        // Members for the device connection
        static string m_deviceIP = "127.0.0.1";    // Localhost if control programm runs on the same machine
        static int m_devicePort = 8080;               // Port for communication with device
        static TcpClient m_deviceClient;              // Tcp socket for connection establishment
        static NetworkStream m_deviceClientStream;    // Datastream of the device
        static Timer m_tryConnectTimer;               // Timer to retry a connection establishment
        static bool m_bConnectedToDevice;             // Flag for device connection
        
        // Members for the consumer connection
        static int m_serverPort = 32323; // 32323
        static TcpListener m_tcpServer;
        static TcpClient m_tcpClientConsumer;
        static NetworkStream m_consumerStream;
        static Timer m_timerAcceptClient;
        static bool m_bConsumerIsConnected;
        
        // Members to configure the downstream access
        static Timer m_downstreamTimer = new System.Timers.Timer(100); // Max length in ms
        static Timer m_maxTimeDownstream = new System.Timers.Timer(60000*5000);
        //static List<List<string>> bigList= new List<List<string>>(10*9);
        static int m_numValidData = 0;
        static int m_numDataRequested  = 0;
         static int countnumber=0;
        static List<string> m_dataAzdtDownload = new List<string>();
        static List<string> m_dataValidIn = new List<string>(); // Addon: stores valid data received from RobotControlProgram


        static List<string> m_downstreamTwins = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", 
                                    "JointPosition5", "JointPosition6", "JointPosition7", "GripperData_Width", "GripperData_Speed"/*,
                                    "Bottle_LocationX", "Bottle_LocationY"*/};
        // ENTRY POINT 
        // JZH: change static async Task Main -> static void Main
        static void Main(string[] args)
        {
            Console.WriteLine("Started the Client-App.\nBeginning with authentication ...\n");
            // Instace URL found in the Azure Portal
            string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";
            // Authenticate with the Azure service
            var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
            DigitalTwinsClient client = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
            m_azureClient = client;
            Console.WriteLine("Service client succsessfully created.");
            
            //------ Setup and Initalization Functions --------
                // This is the ETCIM-1 instance
                // JZH: comment initialize consumer connection
            initialize_consumer_connection(); // Consumer is Isaac Sim 

            // JZH: end modification
            initialize_device_connection(); 
            //-------------------------------------------------
            
            //----- Specify conditions for communication start -----

            // // JZH
            // while( !m_bConnectedToDevice || !m_bConsumerIsConnected ) { 
            //     // Console.WriteLine("Waiting until conditions are satisfied");
            //     await Task.Delay(1000);}
            //    // N.B.: We use a slow poll loop instead of a busy-wait loop which is possible 
            //    //       because the main method is specified as a async task.
            //------------------------------------------------------ 
            // JZH end modifictiaon

            //------ Communication Functions -----------------
            //process_and_forward_fci_data();
            //log_live_data();
            //update_component_test_function();

            // JZH: addd print out...
            Console.WriteLine(m_bConnectedToDevice);
            // JZH: end modifcation
            
            process_and_upload_fci_data();
          
            //process_and_forward_to_simulation();
            //-------------------------------------------------
            
            Console.WriteLine("Press Enter to end program ...");
            Console.ReadLine();
        }
        
        // Establishes connection with a tcp client, for instance the panda robot
        static void initialize_device_connection()
        {
            m_bConnectedToDevice = false;
            call_reconnect();
            // JZH don't need timer
            // m_tryConnectTimer = new System.Timers.Timer(5000);
            // m_tryConnectTimer.Elapsed += call_reconnect;
            // m_tryConnectTimer.Start();
            // JZH: end modification
        }

        // Calls the function for connection establishment
        // JZH: remove input argument Object source, ElapsedEventArgs e
        static void call_reconnect(){
            // JZH change if to while
            while(!m_bConnectedToDevice)
            {
                try_device_connection();
            }

        }

        // Creates the tcp client and network stream for the device
        static void try_device_connection()
        {   
            Console.WriteLine("[INFO] Trying to connect to device.");
            try
            {
                // Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                // IPAddress subscriberAddr = IPAddress.Parse("10.157.175.17");
                // int subscriberPort = SUBSCRIBER_PORT;
                // IPEndPoint endPoint = new IPEndPoint(subscriberAddr, subscriberPort);
                // socket.Bind(endPoint);

                
                m_deviceClient = new TcpClient(m_deviceIP, m_devicePort);
                m_deviceClientStream = m_deviceClient.GetStream();
                m_bConnectedToDevice = true;
                Console.WriteLine("[INFO] Connected with device.------------------------------------------------------------------------------------");
                //if(m_bConnectedToDevice && m_bConsumerIsConnected)
                //    process_and_forward_fci_data();

                // JZH: added if condition
                if (m_bConnectedToDevice)
                {
                    Console.WriteLine(m_bConnectedToDevice);
                }
            }
            catch(SocketException e)
            {
                Console.WriteLine("[WARN] Socket error occured.");
            }
            catch
            {
                Console.WriteLine("[WARN] Error in connection establishment occured.");
            }       
        }

        // -------------------------------------------------------
        static void initialize_consumer_connection()
        {
            // JZH: added assign value m_bConsumerIsConnected to false
            m_bConsumerIsConnected = false;
            // end modification
            m_tcpServer = new TcpListener(IPAddress.Any, m_serverPort);
            m_tcpServer.Start();
          
            

            // JZH: remove timer and added call_accept
            call_accept();

            // m_timerAcceptClient = new System.Timers.Timer(5000);
            // m_timerAcceptClient.Elapsed += call_accept;
            // m_timerAcceptClient.Start();
            // JZH: end modificaiton

        }

        // JZH: removed input arguments: Object source, ElapsedEventArgs e
        static void call_accept(){
            // JZH: change if to while
            while (!m_bConsumerIsConnected)
            {
                try_accept_consumer();
                if (m_bConsumerIsConnected)
                {
                    break;
                }
            }
                
        }

        static void try_accept_consumer()
        {
            try
            {
                m_tcpClientConsumer = m_tcpServer.AcceptTcpClient();
                m_consumerStream = m_tcpClientConsumer.GetStream();
                m_bConsumerIsConnected = true;
                Console.WriteLine("[INFO] Consumer is connected1-------------------------------------------------------------------------");
                //if(m_bConnectedToDevice && m_bConsumerIsConnected)
                //    process_and_forward_fci_data();
            }
            catch
            {
                 Console.WriteLine("[WARN] Error in consumer connection establishment.");
            }
        }

 
        static void process_and_upload_fci_data()
        {
            Console.WriteLine("[INFO] Beginn reading, processing and uploading of device stream.");

            // Analysis variables
            int average_package_size = 25000000;     
            int feasible_read_interval_time = 1; 
            // JZH: assign a smaller buffersize to 1024
            // int necessary_buffer_size = 10*average_package_size*feasible_read_interval_time;
            int necessary_buffer_size = 1024;
            // JZH: end modification

            // Define buffer for the data in both raw bytes and ascii format
            Byte[] fci_data_bytes = new Byte[necessary_buffer_size]; // IMPORTANT TO KNOW THE SIZE HERE
            String fci_data_ascii = String.Empty;
            Int32 num_bytes_read = 0;
            
            // Declare the list of to be updated twins before the read data functions is assigned
            List<string> updateTwinIds = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5", "JointPosition6", "JointPosition7",
                                                             "GripperData_Width", "GripperData_Speed"};

            // JZH: remove timer and use funtion readData()
            while (m_bConnectedToDevice)
            {
                readData();
            }

            // bool continueReading = true; // Flag that can stop the reading process
            // Timer maxReadTestTimer = new System.Timers.Timer(120000); // Timer for whole reading process [ms]
            // maxReadTestTimer.Elapsed += breakReadLoop; // Connect timer to event that sets stop flag
            // maxReadTestTimer.Enabled = true; // Start timer
            // Timer timerReadData = new System.Timers.Timer(feasible_read_interval_time);
            
            // timerReadData.Elapsed += readData;
            // timerReadData.Enabled = true;

            // end modifictaion
            
            // Variables for stream data processing
            int indexOpen = 0;
            int indexClosed = 0;
            JsonDocument genericJsonObject;
            JsonElement root;
            JsonElement time; // Buffer
            JsonElement q; // Buffer for the joint positions
            string log_string = String.Empty; // Used if same string is used mutiple times in read function
            
            // Sets flag that stops reading of the data stream
            // JZH: don't user timer
            // void breakReadLoop(Object source, ElapsedEventArgs e){
            //     continueReading = false;
            //     maxReadTestTimer.Stop();
            //     maxReadTestTimer.Dispose();
            // };
            // JZH: end modication

            // JZH: removed async before void, removed input arguments Object source, ElapsedEventArgs e
            void readData()
            {
                // JZH: remove timer
                // if (!continueReading)
                // {
                //     timerReadData.Enabled = false;
                //     return;
                // }

                // End modification
                // Read data from the socket 
                 countnumber +=1;
                num_bytes_read = m_deviceClientStream.Read(fci_data_bytes, 0, fci_data_bytes.Length);
                //Console.WriteLine(num_bytes_read);
                
                // And convert to ascii string
                fci_data_ascii = System.Text.Encoding.ASCII.GetString(fci_data_bytes, 0, num_bytes_read);
                //Console.WriteLine(fci_data_ascii);
                
                // In this asci string we need to extract the robot state data inside two '{' '}'
                indexOpen = fci_data_ascii.IndexOf('{');
                indexClosed = fci_data_ascii.IndexOf('}');
                //Console.WriteLine(indexOpen.ToString(), indexClosed.ToString());
                
                // If there is a Json object on it we try to parse it
                // By the if condition we ensure that a valid object exits
                if(indexClosed > indexOpen && indexOpen != -1)
                {   
                    m_numValidData += 1;
                    int funcCount = m_numValidData;
                    // Addon Start timing Accessing time
                    string ts_received = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");

                    //Console.WriteLine("Got new valid Data "+ funcCount+" times");

                    // Parse a JSON object from the ascii data, this is fast enough 
                    //----------------------------------
                    // if (indexOpen >= 0 && indexClosed < fci_data_ascii.Length)
                    // {
                      
                        genericJsonObject = JsonDocument.Parse(fci_data_ascii.Substring(indexOpen, indexClosed+1));
                        root = genericJsonObject.RootElement;
                        

                        
                        // Note: indexOpen+7 lets us skip this part: {"q": [
                        // Note: indexClosed lets is skip: ]} compared to indexClosed+1

                        // Create a new patch list for every processed data package
                        List<JsonPatchDocument> patchList = new List<JsonPatchDocument>();
                        if(root.TryGetProperty("q", out q)){
                            // Create a JSON-Patch Document for all Joint Values
                            List<string> dataList = new List<string>();
                        foreach (JsonElement item in q.EnumerateArray())
                        {
                            dataList.Add(item.GetDouble().ToString());
                        }
                       // bigList.Add(dataList);
                        //Console.WriteLine("Big List:"+bigList.Count);
                        // foreach (var list in bigList)
                        // {

                        //     Console.WriteLine("list in biglist:[ " + string.Join(", ", list) + " ]");

                        // }
                        // for (int i = 0; i < 30000; i++)
                        // {
                            // string twinId = updateTwinIds[i % updateTwinIds.Count];
                        List<string> valuesToSend = dataList; // [1,2,3,4,5,6,7]
                        string component_value_result = String.Empty; 
                        BasicDigitalTwin twin_value; 
                        string buffer = "[";
                      

                        for (int j = 0; j < valuesToSend.Count; j++)
                        {   
                            string twinId = updateTwinIds[j % updateTwinIds.Count];//[jointpos1]
                            string value = valuesToSend[j];                //0.258

                            var patch = new JsonPatchDocument();
                            patch.AppendAdd("/value", value);
                             
                            m_azureClient.UpdateDigitalTwin(twinId, patch);
                            // await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);
    

                            //---------------------------------------------------------------------------------------------------
                            
                            // Console.WriteLine("uploaded" +twinId + value );

                            Response<BasicDigitalTwin> getTwinResponse = m_azureClient.GetDigitalTwin<BasicDigitalTwin>(twinId);
                            // Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);

                            twin_value = getTwinResponse.Value;
                            component_value_result = twin_value.Contents["value"].ToString();
                            
                            // JZH: write if condition, if the downloaed value equals to uploaded value, save the value in the buffer
                            // the main bugs comes from here, C# upload and download data async
                            if (component_value_result == value)
                            {
                                buffer += component_value_result; //[0.258
                                buffer += ",";  //[0.258,
                            }
                            else
                            {
                                buffer += value; //[0.258
                                buffer += ",";  //[0.258,
                            }
                            // JZH: end modification
                            Console.WriteLine("downloaded" +buffer );
                        }
                        buffer = buffer.Remove(buffer.Length -1); // remove last comma
                        buffer += "]";    //buffer=[1,2,3,4,5,6]
                        Console.WriteLine("numder of transfer data:" + countnumber);
                        m_dataAzdtDownload.Add(buffer.Substring(1, buffer.Length -2));
                            byte[] byte_buffer = new byte[100]; 
                        byte_buffer = Encoding.Default.GetBytes(buffer);  // convert string to an byte array
                        m_consumerStream.Write(byte_buffer, 0, byte_buffer.Length); 
                        
                    
                    // }
                  
                }   
            }       
        

        /* Actually call the Azure-DT SDK update component method.
         * Note that if a twin is specified incorrectly e.g. by a wrong twin-id then the update proccess
         * will stop at this twin, even if the following twins in the list are still valid.
         */
         static void update_twin_from_patches(List<JsonPatchDocument> patches, List<string> twinIds, int funcCount){
            Console.WriteLine("Displaying Patches");
            Console.WriteLine(patches);
            // update the whole twinID as a whole Sting
            for(int i=0; i<patches.Count; i++){
                m_azureClient.UpdateComponent(twinIds[i], "value", patches[i]); 

            }
            Console.WriteLine("Twin graph updated for Num: " + funcCount);
        }
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
            static void call_downstream_access(Object source, ElapsedEventArgs e){
                 donwstream_access();
                
                Console.WriteLine("[INFO] Downstream is sucessfully accessed");
            }

            //@brief Implements our azure downstream, i.e. it gets the relevant data from the digitial
            //        twin instance and sends it to further processes, e.g. omniverse.
            static void donwstream_access()
            {
                int numDataRequested = m_numDataRequested;
                m_numDataRequested++;
                string component_value_result = String.Empty;          
                BasicDigitalTwin twin_value; 
                string buffer = "[";
                        
                foreach (var joint_twin in m_downstreamTwins)
                {

                    Response<BasicDigitalTwin> getTwin_Response = m_azureClient.GetDigitalTwin<BasicDigitalTwin>(joint_twin);
                    twin_value = getTwin_Response.Value;
                    
                    component_value_result = twin_value.Contents["value"].ToString();

                    Console.WriteLine("[INFO] Started accessing the Azure downstream"+twin_value);
                   // Console.WriteLine(component_value_result);
                    buffer += component_value_result;
                    buffer += ",";
                }
                buffer = buffer.Remove(buffer.Length -1);
                buffer += "]";
                m_dataAzdtDownload.Add(buffer.Substring(1, buffer.Length -2));
                Console.WriteLine(buffer);


                byte[] byte_buffer = new byte[100]; 
                byte_buffer = Encoding.Default.GetBytes(buffer);  // convert string to an byte array
                m_consumerStream.Write(byte_buffer, 0, byte_buffer.Length);     //sending the message
                //Console.WriteLine("[INFO] Downloaded and Forwarded data " + numDataRequested + " times");
            }

            // Clean up all timers and close the connections
            void stop_downstream_access(Object source, ElapsedEventArgs e){
                m_downstreamTimer.Stop();
                m_downstreamTimer.Dispose();
                m_maxTimeDownstream.Stop();
                m_maxTimeDownstream.Dispose();
                m_consumerStream.Close();
                m_tcpClientConsumer.Close();
                Console.WriteLine("[END] Finished downstream access");
            }
        }






        }

        /* Verifies that updating a specific component of the twin instance works as expected.
         * Thereby verifies the connection to the twin instance.
         */
        
        /* 
         *  Access the data from the Azure Digital Twins instance, create an array of the joint values
         *  and forward it to the simulation.
         */
        
        // @brief Stores all recieved json packages in a file for replay purposes.
        static void log_data(string json_data_string, string log_file_name="SomeLog"){
            using(StreamWriter log_stream = File.AppendText(log_file_name+".txt"))
            {
                string timestamp = DateTime.Now.ToString("hh.mm.ss.ffffff");
                log_stream.WriteLine(timestamp+" "+json_data_string);
            }
        }
}
}}

