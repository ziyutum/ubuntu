﻿/* EXTENSIBLE TWIN COMMUNICATION INTERFACE MODULE (ETCIM)
 * This program presents the central interface between the physical assset
 * and its virtual representation. 
 * 
 * Documentation of Azure Digital Twins SDK:
 * https://docs.microsoft.com/de-de/dotnet/api/azure.digitaltwins.core?view=azure-dotnet
 * Doc Azure Digital Twins REST API:
 * https://docs.microsoft.com/de-DE/rest/api/azure-digitaltwins/
 * 
 * Author: Josua Hoefgen
 */

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

        static int m_numValidData = 0;
        static int m_numDataRequested  = 0;
        static List<string> m_dataAzdtDownload = new List<string>();
        static List<string> m_dataValidIn = new List<string>(); // Addon: stores valid data received from RobotControlProgram


        static List<string> m_downstreamTwins = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", 
                                    "JointPosition5", "JointPosition6", "JointPosition7", "GripperData_Width", "GripperData_Speed"/*,
                                    "Bottle_LocationX", "Bottle_LocationY"*/};
        // ENTRY POINT 
        static async Task Main(string[] args)
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
           // initialize_consumer_connection(); // Consumer is Isaac Sim 
            initialize_device_connection(); 
            //-------------------------------------------------
            
            //----- Specify conditions for communication start -----
            while( !m_bConnectedToDevice /*|| !m_bConsumerIsConnected */ ) { 
                // Console.WriteLine("Waiting until conditions are satisfied");
                await Task.Delay(1000);}
               // N.B.: We use a slow poll loop instead of a busy-wait loop which is possible 
               //       because the main method is specified as a async task.
            //------------------------------------------------------ 

            //------ Communication Functions -----------------
            //process_and_forward_fci_data();
            //log_live_data();
            //update_component_test_function();
            process_and_upload_fci_data();
            //downstream_simulation_forwarding();
            //process_and_forward_to_simulation();
            //-------------------------------------------------
            
            Console.WriteLine("Press Enter to end program ...");
            Console.ReadLine();
        }
        
        // Establishes connection with a tcp client, for instance the panda robot
        static void initialize_device_connection()
        {
            m_bConnectedToDevice = false;
            m_tryConnectTimer = new System.Timers.Timer(5000);
            m_tryConnectTimer.Elapsed += call_reconnect;
            m_tryConnectTimer.Start();
        }

        // Calls the function for connection establishment
        static void call_reconnect(Object source, ElapsedEventArgs e){
            if(!m_bConnectedToDevice)
                try_device_connection();
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
                Console.WriteLine("[INFO] Consumer is connected1-------------------------------------------------------------------------");
                //if(m_bConnectedToDevice && m_bConsumerIsConnected)
                //    process_and_forward_fci_data();
            }
            catch
            {
                 Console.WriteLine("[WARN] Error in consumer connection establishment.");
            }
        }


        /*  Reads and deserializes the device data stream
         *  and extracts relevant data to subsequently
         *  call the functions that create the JSON Patch Docuemtns to update the twin instance.
         */
        static void process_and_upload_fci_data()
        {
            Console.WriteLine("[INFO] Beginn reading, processing and uploading of device stream.");

            // Analysis variables
            int average_package_size = 2500;     
            int feasible_read_interval_time = 10; 
            int necessary_buffer_size = average_package_size*feasible_read_interval_time;

            // Define buffer for the data in both raw bytes and ascii format
            Byte[] fci_data_bytes = new Byte[necessary_buffer_size]; // IMPORTANT TO KNOW THE SIZE HERE
            String fci_data_ascii = String.Empty;
            Int32 num_bytes_read = 0;
            
            // Declare the list of to be updated twins before the read data functions is assigned
            List<string> updateTwinIds = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5", "JointPosition6", "JointPosition7",
                                                             "GripperData_Width", "GripperData_Speed"};

            bool continueReading = true; // Flag that can stop the reading process
            Timer maxReadTestTimer = new System.Timers.Timer(1000000); // Timer for whole reading process [ms]
            maxReadTestTimer.Elapsed += breakReadLoop; // Connect timer to event that sets stop flag
            maxReadTestTimer.Enabled = true; // Start timer
            Timer timerReadData = new System.Timers.Timer(feasible_read_interval_time);
            
            timerReadData.Elapsed += readData;
            timerReadData.Enabled = true;
            
            // Variables for stream data processing
            int indexOpen = 0;
            int indexClosed = 0;
            JsonDocument genericJsonObject;
            JsonElement root;
            JsonElement time; // Buffer
            JsonElement q; // Buffer for the joint positions
            string log_string = String.Empty; // Used if same string is used mutiple times in read function
            
            // Sets flag that stops reading of the data stream
            void breakReadLoop(Object source, ElapsedEventArgs e){
                continueReading = false;
                maxReadTestTimer.Stop();
                maxReadTestTimer.Dispose();
            };
                 
             async void readData(Object source, ElapsedEventArgs e)
            {

                if (!continueReading)
                {
                    timerReadData.Enabled = false;
                    return;
                }

                // Read data from the socket 
                num_bytes_read = await m_deviceClientStream.ReadAsync(fci_data_bytes, 0, fci_data_bytes.Length);
                Console.WriteLine(num_bytes_read);
                  
                if (num_bytes_read > 0){
                // // And convert to ascii string
                fci_data_ascii = System.Text.Encoding.ASCII.GetString(fci_data_bytes, 0, num_bytes_read);
                Console.WriteLine("Received data as ASCII string:"+fci_data_ascii);
                //fci_data_ascii is {"q": [0.037391,-0.839807,0.002659,-2.395532,0.001890,1.556708,0.789682,0.0200000000000000,0.0200000]}
                //Received data as ASCII string:{"q": [-0.853044,-0.011639,-0.000394,-2.893954,-0.000404,2.881984,-0.067567,0.02000000000000,0.020000]}
// {"q": [-0.853043,-0.011819,-0.000389,-2.893958,-0.000403,2.881816,-0.067568,0.02000000000000,0.020000]}
// {"q": [-0.853047,-0.011871,-0.000393,-2.893971,-0.000403,2.881755,-0.067572,0.02000000000000,0.020000]}
// {"q": [-0.853044,-0.011929,-0.000393,-2.893967,-0.000402,2.881696,-0.067566,0.02000000000000,0.020000]}
// {"q": [-0.853041,-0.011992,-0.000393,-2.893972,-0.000402,2.881636,-0.067566,0.02000000000000,0.020000]}
// {"q": [-0.853044,-0.012053,-0.000396,-2.893975,-0.000403,2.881576,-0.067566,0.02000000000000,0.020000]}
// {"q": [-0.853046,-0.012110,-0.000396,-2.893980,-0.000406,2.881518,-0.067565,0.02000000000000,0.020000]}
// {"q": [-0.853041,-0.012168,-0.000396,-2.893984,-0.000406,2.881461,-0.067569,0.02000000000000,0.020000]}
// {"q": [-0.853046,-0.012231,-0.000396,-2.893990,-0.000406,2.881404,-0.067570,0.02000000000000,0.020000]}
// {"q": [-0.853045,-0.012301,-0.000394,-2.893996,-0.000404,2.881351,-0.067567,0.02000000000000,0.020000]}
// {"q": [-0.853044,-0.012358,-0.000396,-2.894002,-0.000402,2.881294,-0.067566,0.02000000000000,0.020000]}
// {"q": [-0.853044,-0.012422,-0.000394,-2.894008,-0.000402,2.881238,-0.067566,0.02000000000000,0.020000]}


                        // Split received data by new line to process each JSON object separately
                    string[] jsonLines = fci_data_ascii.Split(new[] {  "\n" }, StringSplitOptions.RemoveEmptyEntries);
                    Console.WriteLine("jsonlines islike "+jsonLines);
                    List<List<string>> bigList = new List<List<string>>();
                    foreach (string jsonLine in jsonLines)
                    { Console.WriteLine("jsonline "+jsonLine);
                        try
                        {
                            // Parse JSON
                            JsonDocument jsonDoc = JsonDocument.Parse(jsonLine);
                            JsonElement root = jsonDoc.RootElement;
                            JsonElement qArray = root.GetProperty("q");
                            List<string> qValues = new List<string>();
                            foreach (JsonElement element in qArray.EnumerateArray())
                            {
                                qValues.Add(element.ToString());
                            }
                            bigList.Add(qValues);
                            // Console.WriteLine("\nExtracted 'q' values:");
                            // foreach (string value in qValues)
                            // {
                            //     Console.WriteLine(value);
                            // }
                         Console.WriteLine("Big List:");
                        foreach (var smallList in bigList)
                        {
                            Console.WriteLine("[" + string.Join(", ", smallList) + "]");
                        }


                            List<string> list2 = new List<string>();

                             // Update twin ids
                            // List<string> updateTwinIds = new List<string>
                            // {
                            //     "JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5","JointPosition6","JointPosition7",
                            //     "GripperData_Width", "GripperData_Speed"
                            // };
                            for(int i=0; i<updateTwinIds.Count; i++)
                            {
                                string twinId = updateTwinIds[i];
                                string value = qValues[i];
                                var patch = new JsonPatchDocument();
                                patch.AppendAdd("/value", value);
                                await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);

                                //m_azureClient.UpdateComponent(updateTwinIdswinIds[i], "value", list1[i]); 
                                // Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);
                                // var twinValue = getTwinResponse.Value;

                                // string componentValueResult = twinValue.Contents["value"].ToString();
                                // list2.Add(componentValueResult);                      
                                }


                            // Optionally, process the qValues list further or perform actions based on the parsed JSON data.

                        }
                        catch (JsonException ex)
                        {
                            Console.WriteLine($"Error parsing JSON: {ex.Message}");
                            // Handle the exception (e.g., log, retry, ignore)
                        }
                    }
                }
                else
                {
                    Console.WriteLine("No data read from the socket.");
                }


                // indexOpen = fci_data_ascii.IndexOf('{');
                // indexClosed = fci_data_ascii.IndexOf('}');
                // //Console.WriteLine(indexOpen.ToString(), indexClosed.ToString());
                
                // // If there is a Json object on it we try to parse it
                // // By the if condition we ensure that a valid object exits
                // if(indexClosed > indexOpen && indexOpen != -1)
                // {   
                //     m_numValidData += 1;
                //     int funcCount = m_numValidData;
                //     // Addon Start timing Accessing time
                //     string ts_received = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");

                //     //Console.WriteLine("Got new valid Data "+ funcCount+" times");

                //     // Parse a JSON object from the ascii data, this is fast enough 
                //     //----------------------------------
                //     if (indexOpen >= 0 && indexClosed < fci_data_ascii.Length)
                //     {
                      
                //         genericJsonObject = JsonDocument.Parse(fci_data_ascii.Substring(indexOpen, indexClosed+1));
                //         root = genericJsonObject.RootElement;
                        

                        
                //         // Note: indexOpen+7 lets us skip this part: {"q": [
                //         // Note: indexClosed lets is skip: ]} compared to indexClosed+1

                //         // Create a new patch list for every processed data package
                //         List<JsonPatchDocument> patchList = new List<JsonPatchDocument>();
                //         if(root.TryGetProperty("q", out q)){
                //             // Create a JSON-Patch Document for all Joint Values
                //             for(int i=0; i<9; i++){
                //                 JsonPatchDocument operational_data_patch = new JsonPatchDocument(); // Create new in every run   
                //                 operational_data_patch.AppendAdd<string>("", q[i].GetDouble().ToString());

                //                 Console.WriteLine("op"+operational_data_patch);//[{"op":"add","path":"","value":"0.835743"}]


                //                 patchList.Add(operational_data_patch);

                //                 //ziyu 
                //                 Console.WriteLine("PATCHLIST "+patchList.ToString());
                //                 foreach (var patch in patchList)
                //                 {
                //                     Console.WriteLine(patch.ToString());        
                //                         //PATCHLIST System.Collections.Generic.List`1[Azure.JsonPatchDocument]
                //                         // [{"op":"add","path":"","value":"0.042734"}]
                //                         // [{"op":"add","path":"","value":"0.427292"}]
                //                         // [{"op":"add","path":"","value":"-0.514319"}]
                //                         // [{"op":"add","path":"","value":"-1.656473"}]
                //                         // [{"op":"add","path":"","value":"0.246766"}]
                //                         // op[{"op":"add","path":"","value":"2.069119"}]
                //                         //                             }
                //                 }
                //             }
                //         }
                //     }
                // }
            }

















                 // In this asci string we need to extract the robot state data inside two '{' '}'
                //  indexOpen = fci_data_ascii.IndexOf('{');
                // indexClosed = fci_data_ascii.IndexOf('}');
                // Console.WriteLine(indexOpen.ToString(), indexClosed.ToString());
                
                // // // If there is a Json object on it we try to parse it
                // // // By the if condition we ensure that a valid object exits
                // if(indexClosed > indexOpen && indexOpen != -1)
                // {   
                //     m_numValidData += 1;
                //     int funcCount = m_numValidData;
                //     // Addon Start timing Accessing time
                //     string ts_received = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");

                //     Console.WriteLine("Got new valid Data "+ funcCount+" times");
                    
                 
                // //     // Parse a JSON object from the ascii data, this is fast enough 
                // //     //----------------------------------
                //     if (indexOpen >= 0 && indexClosed < fci_data_ascii.Length)
                //     {
                      
                // JsonDocument genericJsonObject = JsonDocument.Parse(fci_data_ascii);//.Substring(indexOpen, indexClosed+1));
                // JsonElement root = genericJsonObject.RootElement;
                // JsonElement qArray = root.GetProperty("q");
                // List<string> qValues = new List<string>();
                // foreach (JsonElement element in qArray.EnumerateArray())
                // {
                //     qValues.Add(element.ToString());
                // }
                // Console.WriteLine("\nExtracted 'q' values:");

                // Console.WriteLine("List of q Values:");
                // foreach (string value in qValues)
                // {
                //     Console.WriteLine(value);
                // }
                        // Note: indexOpen+7 lets us skip this part: {"q": [
                        // Note: indexClosed lets is skip: ]} compared to indexClosed+1

                        // Create a new patch list for every processed data package
                        //List<JsonPatchDocument> patchList = new List<JsonPatchDocument>();
                        // if(root.TryGetProperty("q", out q)){
                        //     // Create a JSON-Patch Document for all Joint Values
                        //     for(int i=0; i<9; i++){
                        //         JsonPatchDocument operational_data_patch = new JsonPatchDocument(); // Create new in every run   
                        //         operational_data_patch.AppendAdd<string>("", q[i].GetDouble().ToString());

                        //         Console.WriteLine("op"+operational_data_patch);//[{"op":"add","path":"","value":"0.835743"}]


                        //         patchList.Add(operational_data_patch);

                        //         //ziyu 
                        //         Console.WriteLine("PATCHLIST "+patchList.ToString());
                        //         foreach (var patch in patchList)
                        //         {
                        //             Console.WriteLine(patch.ToString());        
                        //                 //PATCHLIST System.Collections.Generic.List`1[Azure.JsonPatchDocument]
                        //                 // [{"op":"add","path":"","value":"0.042734"}]
                        //                 // [{"op":"add","path":"","value":"0.427292"}]
                        //                 // [{"op":"add","path":"","value":"-0.514319"}]
                        //                 // [{"op":"add","path":"","value":"-1.656473"}]
                        //                 // [{"op":"add","path":"","value":"0.246766"}]
                        //                 // op[{"op":"add","path":"","value":"2.069119"}]
                        //                 //                             }
                        //         }
                        //     }
                    
                    // if (funcCount % 80 == 0)
                    // {
                    //     await update_twin_from_patches_downloading(patchList, updateTwinIds, funcCount);
                    //     Console.WriteLine("Started upload data for "+ funcCount);
                    //     Console.WriteLine("Downsteaming the values in the ID ");
                        
                    //     //downstream_simulation_forwarding();
                    // }
                    
                    // }
                  
                // }   
            

        /* Actually call the Azure-DT SDK update component method.
         * Note that if a twin is specified incorrectly e.g. by a wrong twin-id then the update proccess
         * will stop at this twin, even if the following twins in the list are still valid.
         */
//          static async Task update_twin_from_patches_downloading(List<JsonPatchDocument> patches, List<string> twinIds, int funcCount){
//             Console.WriteLine("Displaying Patches");
//             Console.WriteLine(patches);
//             // update the whole twinID as a whole Sting
//             for(int i=0; i<patches.Count; i++){
//             string twinId = twinIds[i % twinIds.Count];
//             JsonPatchDocument patch = patches[i];

//             // var patch = new JsonPatchDocument();
//             // patch.AppendAdd("/value", JsonSerializer.Serialize(valuesToSend)); // 
//             await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);

            
//             Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);
//             var twinValue = getTwinResponse.Value;

//             // 
//             string componentValueResult = twinValue.Contents["value"].ToString();

//             //
//             List<string> componentValues = JsonSerializer.Deserialize<List<string>>(componentValueResult);
//             Console.WriteLine("ListOmniverse_component:"+componentValues);

//             byte[] byteBuffer = Encoding.UTF8.GetBytes(string.Join(",", componentValues));


//             //byte_buffer = Encoding.Default.GetBytes(buffer);  // convert string to an byte array
//             m_consumerStream.Write(byteBuffer, 0, byteBuffer.Length); 
//             foreach (var values in componentValues)
//             {
//                 Console.WriteLine(string.Join(", ", values));
//             }
//                // m_azureClient.UpdateComponent(twinIds[i], "value", patches[i]); 

//             }
//             //Console.WriteLine("Twin graph updated for Num: " + funcCount);
//         }
//         static void downstream_simulation_forwarding()
//         {
//             // Initialize the downstream access timers
//             m_downstreamTimer.Elapsed += call_downstream_access;   // Function to handle timer event
//             m_downstreamTimer.Enabled = true;                      // Start the timer 
//             m_maxTimeDownstream.Elapsed += stop_downstream_access; // Clean up after timeout
//             m_maxTimeDownstream.Enabled = true;                    // Start max connection timer
//             Console.WriteLine("[INFO] Started accessing the Azure downstream");

//             //@brief Called when the timer for downstream access elapsas and handels
//             //        the actual call to the methods that accesses Azure.
//             static void call_downstream_access(Object source, ElapsedEventArgs e){
//                  donwstream_access();
                
//                 Console.WriteLine("[INFO] Downstream is sucessfully accessed");
//             }

//             //@brief Implements our azure downstream, i.e. it gets the relevant data from the digitial
//             //        twin instance and sends it to further processes, e.g. omniverse.
//             static void donwstream_access()
//             {
//                 int numDataRequested = m_numDataRequested;
//                 m_numDataRequested++;
//                 string component_value_result = String.Empty;          
//                 BasicDigitalTwin twin_value; 
//                 string buffer = "[";
                        
//                 foreach (var joint_twin in m_downstreamTwins)
//                 {

//                     Response<BasicDigitalTwin> getTwin_Response = m_azureClient.GetDigitalTwin<BasicDigitalTwin>(joint_twin);
//                     twin_value = getTwin_Response.Value;
                    
//                     component_value_result = twin_value.Contents["value"].ToString();

//                     Console.WriteLine("[INFO] Started accessing the Azure downstream"+twin_value);
//                    // Console.WriteLine(component_value_result);
//                     buffer += component_value_result;
//                     buffer += ",";
//                 }
//                 buffer = buffer.Remove(buffer.Length -1);
//                 buffer += "]";
//                 m_dataAzdtDownload.Add(buffer.Substring(1, buffer.Length -2));
//                 Console.WriteLine(buffer);


//                 byte[] byte_buffer = new byte[100]; 
//                 byte_buffer = Encoding.Default.GetBytes(buffer);  // convert string to an byte array
//                 m_consumerStream.Write(byte_buffer, 0, byte_buffer.Length);     //sending the message
//                 //Console.WriteLine("[INFO] Downloaded and Forwarded data " + numDataRequested + " times");
//             }

//             // Clean up all timers and close the connections
//             void stop_downstream_access(Object source, ElapsedEventArgs e){
//                 m_downstreamTimer.Stop();
//                 m_downstreamTimer.Dispose();
//                 m_maxTimeDownstream.Stop();
//                 m_maxTimeDownstream.Dispose();
//                 m_consumerStream.Close();
//                 m_tcpClientConsumer.Close();
//                 Console.WriteLine("[END] Finished downstream access");
//             }
//         }




//                 }

// }

//         /* Verifies that updating a specific component of the twin instance works as expected.
//          * Thereby verifies the connection to the twin instance.
//          */
        
//         /* 
//          *  Access the data from the Azure Digital Twins instance, create an array of the joint values
//          *  and forward it to the simulation.
//          */
        
//         // @brief Stores all recieved json packages in a file for replay purposes.
        
        }
    }
}
