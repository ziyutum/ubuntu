/* 
 * Documentation of Azure Digital Twins SDK:
 * https://docs.microsoft.com/de-de/dotnet/api/azure.digitaltwins.core?view=azure-dotnet
 * Doc Azure Digital Twins REST API:
 * https://docs.microsoft.com/de-DE/rest/api/azure-digitaltwins/
 
 */
// new version 24,05,2024
using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Text.Json; // For deserialization of the dtdl description of the twin instances and for deserialization of operational data to objects where we don't have a defined type
using System.Timers; // To define fixed time intervalls when twin is updated or data is from twin is read

// Azure, Azure Digital Twins realted imports
//       https://docs.microsoft.com/de-de/azure/digital-twins/tutorial-code
// we are using netcore version 8 on ubuntu, check the config before running to ensure the NetCore version is 8.0

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
        //static string m_deviceIP = "10.183.240.78";    // Local IP-Address of machine running RobotControlProgram (device)
        static string m_deviceIP = "127.0.0.1";                  // use "Localhost" it is the same machine
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
        static List<List<string>> bigList= new List<List<string>>(200000);
        static List<string> m_downstreamTwins = new List<string>{"PositionJoint1", "PositionJoint2", "PositionJoint3", "PositionJoint4", "PositionJoint5", "PositionJoint6", "PositionJoint7"};
        
        // Vars for debugging
        static int m_numValidData = 0;
        static List<string> m_dataValidIn = new List<string>(); // Addon: stores valid data received from RobotControlProgram



        // ENTRY POINT 
        static async Task Main(string[] args)
        {
            Console.WriteLine("Started the Client-App");
            Console.WriteLine("Beginning with authentication ...");

            // Instace URL found in the Azure Portal
            string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";
            // https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net        https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net
            // Authenticate with the Azure service
            var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
            DigitalTwinsClient client = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
            m_azureClient = client;
            Console.WriteLine("[SUCCESS] Authentication finished ");
            
            
            initialize_device_connection(); // device means machine runing CSClient program
            // This is the ETCIM-1 instance
            //initialize_consumer_connection(); // Consumer is Isaac Sim
    
            //----- Specify conditions for communication start -----
            while( !m_bConnectedToDevice   /*|| !m_bConsumerIsConnected */) { 
                // Console.WriteLine("Waiting until conditions are satisfied");
                await Task.Delay(1000);}
               // N.B.: We use a slow poll loop instead of a busy-wait loop which is possible 
               //       because the main method is specified as a async task.
            //------------------------------------------------------ 
            
            process_and_upload_fci_data();
            
            Console.WriteLine("Press Enter to end program ...");
            Console.ReadLine();
        }
        
        // Setup loop to try connection 
        static void initialize_device_connection()
        {
            m_bConnectedToDevice = false;
            m_tryConnectTimer = new System.Timers.Timer(5000);
            m_tryConnectTimer.Elapsed += call_reconnect;
            m_tryConnectTimer.Start();
        }
        static void call_reconnect(Object source, ElapsedEventArgs e){
            if(!m_bConnectedToDevice)
                try_device_connection();
        }
        // ---- END ---


        // Creates the tcp client and network stream for the device
        static void try_device_connection()
        {   
            Console.WriteLine("[INFO] Trying to connect to device with IP:" + m_deviceIP);
            try
            {
                m_deviceClient = new TcpClient(m_deviceIP, m_devicePort);
                m_deviceClientStream = m_deviceClient.GetStream();
                m_bConnectedToDevice = true;
                Console.WriteLine("[SUCCESS] Connected with device.");
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

        /*  -------------  BUSINESS LOGIC  --------------
            Reads and deserializes the device data stream
         *  and extracts relevant data to subsequently
         *  call the functions that create the JSON Patch Docuemtns to update the twin instance.
         */
        static void process_and_upload_fci_data()
        {

            Console.WriteLine("[INFO] Beginn reading, processing and uploading of device stream.");

            // Analysis variables
            int average_package_size = 25000000;     
            int feasible_read_interval_time = 1; 
            int necessary_buffer_size = 10*average_package_size*feasible_read_interval_time;

            // Define buffer for the data in both raw bytes and ascii format
            Byte[] fci_data_bytes = new Byte[necessary_buffer_size]; // IMPORTANT TO KNOW THE SIZE HERE
            String fci_data_ascii = String.Empty;
            Int32 num_bytes_read = 0;
            
            // Declare the list of to be updated twins before the read data functions is assigned
            // gripper closing width and gripper opening width are wrong, they should be the gripper left & right finger width
            List<string> updateTwinIds = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5", "JointPosition6", "JointPosition7",
                                                             "GripperData_Width", "GripperData_Speed"};

            bool continueReading = true; // Flag that can stop the reading process
            Timer timerMaxRead = new System.Timers.Timer(120000); // Timer for whole reading process [ms]
            timerMaxRead.Elapsed += breakReadLoop; // Connect timer to event that sets stop flag
            timerMaxRead.Enabled = true; // Start timer

            Timer timerReadData = new System.Timers.Timer(feasible_read_interval_time);
            int countcalling=0;
            timerReadData.Elapsed += readData;
            
            timerReadData.Enabled = true;
            
            
            // Variables for stream data processing
            int indexOpen = 0;
            int indexClosed = 0;
            JsonDocument genericJsonObject;
            JsonElement root;
            JsonElement time; // Buffer
            JsonElement q; // joint positions
            string log_string = String.Empty; // Used if same string is used mutiple times in read function
            
            // Sets flag that stops reading of the data stream
            void breakReadLoop(Object source, ElapsedEventArgs e){
                Console.WriteLine("Breaking Read Loop");
                continueReading = false;
                timerMaxRead.Stop();
                timerMaxRead.Dispose();

                // Addon: Exporting mechanism for received data
            //     string dataFilePath = "data_export/valid_input.csv";
            //     using (StreamWriter writer = new StreamWriter(dataFilePath))
            //     {
            //         writer.WriteLine("ts_in,ts_out,"+string.Join(",",updateTwinIds));   
            //         foreach (string str in m_dataValidIn)
            //         {
            //             writer.WriteLine(str);
            //         }
            //     }
            //     Console.WriteLine($"[SUCCESS] Exported recorded data to {dataFilePath} (Datapoints: {m_dataValidIn.Count})");
             }
                 
            void readData(Object source, ElapsedEventArgs e)
            {
                if (!continueReading)
                {
                    timerReadData.Enabled = false;
                    return;
                }

                // Read data from the socket 
                
                num_bytes_read = m_deviceClientStream.Read(fci_data_bytes, 0, fci_data_bytes.Length);
                //countcalling +=1;

                Console.WriteLine(num_bytes_read);
                Console.WriteLine("callingnumber"+countcalling);
                
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
                    genericJsonObject = JsonDocument.Parse(fci_data_ascii.Substring(indexOpen, indexClosed+1));
                    root = genericJsonObject.RootElement;
                    

                    
                    // Note: indexOpen+7 lets us skip this part: {"q": [
                    // Note: indexClosed lets is skip: ]} compared to indexClosed+1

                    // Create a new patch list for every processed data package
                    List<JsonPatchDocument> patchList = new List<JsonPatchDocument>();
                    if(root.TryGetProperty("q", out q))
                    {
                        List<string> dataList = new List<string>();
                        foreach (JsonElement item in q.EnumerateArray())
                        {
                            dataList.Add(item.GetDouble().ToString());
                        }
                        for (int j = 0; j < dataList.Count; j++)
                            {
                                string twinId = updateTwinIds[j % updateTwinIds.Count];//[jointpos1]
                                string value = dataList[j];                //0.258
                                


                                var patch = new JsonPatchDocument();
                                patch.AppendAdd("/value", value);
                                m_azureClient.UpdateDigitalTwin(twinId, patch);
                            }







                        // bigList.Add(dataList);
                        // Console.WriteLine("Big List:"+bigList.Count);
                        // foreach (var list in bigList)
                        // {

                        //     Console.WriteLine("[ " + string.Join(", ", list) + " ]");

                        // }
                        //__________________________big list generation
                        // for (int i = 0; i < 10000; i++)
                        // {
                        //     string twinId = updateTwinIds[i % updateTwinIds.Count];
                        //     List<string> valuesToSend = bigList[i];
                        //     for (int j = 0; j < valuesToSend.Count; j++)
                        //     {
                        //         //string twinId = updateTwinIds[j % updateTwinIds.Count];//[jointpos1]
                        //         string value = valuesToSend[j];                //0.258
                        //         Console.WriteLine("vale:"+ value);


                        //         var patch = new JsonPatchDocument();
                        //         patch.AppendAdd("/value", value);
                        //         m_azureClient.UpdateDigitalTwin(twinId, patch);
                        //     }

                        //     // var patch = new JsonPatchDocument();
                        //     // patch.AppendAdd("/value", JsonSerializer.Serialize(valuesToSend));
                        //     // m_azureClient.UpdateDigitalTwinAsync(twinId, patch);
                        // }






                        // for (int i = 0; i < 100000; i++)
                        // {
                        //  //string twinId = updateTwinIds[i % updateTwinIds.Count];
                            
                        //  List<string> valuesToSend = bigList[i]; // [1,2,3,4,5,6,7]
                        //     // string component_value_result = String.Empty; 
                        //     // BasicDigitalTwin twin_value; 
                        //     // string buffer = "[";

                        //     for (int j = 0; j < valuesToSend.Count; j++)
                        //     {
                        //         string twinId = updateTwinIds[j % updateTwinIds.Count];//[jointpos1]
                        //         string value = valuesToSend[j];                //0.258
                        //         //Console.WriteLine("vale:"+ value);


                        //         var patch = new JsonPatchDocument();
                        //         patch.AppendAdd("/value", value);
                        //         m_azureClient.UpdateDigitalTwin(twinId, patch);
                        //     }
                        // }

                                //---------------------------------------------------------------------------------------------------


                    // string ts_uploaded = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");
           

                     }   //if
                 }    //if  
            

     
    
             } //void
        
        } //void proces fci
    }//program


}  //name