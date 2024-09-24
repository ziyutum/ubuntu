using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Text;
using System.Text.Json; // For deserialization of the dtdl description of the twin instances and for deserialization of operational data to objects where we don't have a defined type
using System.Timers; // To define fixed time intervals when twin is updated or data is from twin is read

// Azure, Azure Digital Twins related imports
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;

class Program
{
    private static List<string> positionMessages = new List<string>();
    private static List<(DateTime timestamp, string position, double x, double y, double angle)> positionList = new List<(DateTime timestamp, string position, double x, double y, double angle)>();
    private static DateTime startTime;
    private static string lastPosition = "";
    private static double x = -0.30094, y = -0.47503, z = 0.91567, angle = 0, angle2 =0, angle3 = 0;
    private static readonly object lockObj = new object();
    private static DigitalTwinsClient m_azureClient;
    private static List<string> updateTwinIds = new List<string> { "Bottle_Position", "Bottle_LocationX", "Bottle_LocationY", "Switch1_Angle" };
    static int m_serverPort = 8080; // Port for server
    static TcpListener m_tcpServer;
    static TcpClient m_tcpClientConsumer;
    static NetworkStream m_consumerStream;
    static bool m_bConsumerIsConnected;
    static System.Timers.Timer m_timerAcceptClient;
    private static bool isTcpConnected = false; // Flag to indicate TCP connection status


    static async Task Main(string[] args)
    {

        //----------------------------------------------------------azure uploading-------------------------------------------------------------------
        // Console.WriteLine("Started the Client-App");
        // Console.WriteLine("Beginning with authentication ...");
        // string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";

        // var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
        // m_azureClient = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
        // Console.WriteLine("[SUCCESS] Authentication finished");

        //-------------------------------------------------------------------------------------------------------------------------------------------------

        var mqttClient = SetupMqttClient();

        var options = new MqttClientOptionsBuilder()
            .WithClientId("CX-2040-Receiver")
            .WithTcpServer("192.168.80.100", 1883)
            .WithCleanSession()
            .Build();

        try
        {
            await mqttClient.ConnectAsync(options, System.Threading.CancellationToken.None);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }

        initialize_consumer_connection();

        // Start the timer for periodic updates
        // System.Timers.Timer updateTimer = new System.Timers.Timer(100); // Update every seconds
        // updateTimer.Elapsed += UpdateTimer_Elapsed;
        // updateTimer.Start();

        // Keep the application running
        Console.WriteLine("Press any key to exit.");
        Console.ReadLine();

        await mqttClient.DisconnectAsync();
    }

    static IMqttClient SetupMqttClient()
    {
        var factory = new MqttFactory();
        var mqttClient = factory.CreateMqttClient();

        mqttClient.UseConnectedHandler(async e =>
        {
            Console.WriteLine("Connected successfully with MQTT Brokers.");

            // Subscribe to the topics
            await mqttClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("MyJoghurt2Panda/ProvideBottle").Build());
            Console.WriteLine("Subscribed to the topics.");
        });

        mqttClient.UseDisconnectedHandler(e =>
        {
            Console.WriteLine("Disconnected from MQTT Brokers.");
        });

        mqttClient.UseApplicationMessageReceivedHandler(e =>
        {
            var payload = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
            var timestamp = DateTime.Now;

            HandleMqttMessage(payload, timestamp);
        });

        return mqttClient;
    }

         static void initialize_consumer_connection()
        {
            m_tcpServer = new TcpListener(IPAddress.Any, m_serverPort);
            m_tcpServer.Start();
            m_timerAcceptClient = new System.Timers.Timer(500);
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
                isTcpConnected = true;
                Console.WriteLine("[INFO] Consumer is connected");
                //if(m_bConnectedToDevice && m_bConsumerIsConnected)
                //    process_and_forward_fci_data();
            }
            catch
            {
                Console.WriteLine("[WARN] Error in consumer connection establishment.");
            }
        }           

    static void HandleMqttMessage(string payload, DateTime timestamp)
    {
        if (!isTcpConnected)
        {
            Console.WriteLine("[INFO] TCP not connected. Ignoring MQTT message.");
            return;
        }
        string currentPosition = null;

        if (payload.StartsWith("The Bottle is : On the Conveyer 1   ,Position : On the Conveyer 1"))
        {
            currentPosition = "ON_CONVEYER_1";
        }
        else if (payload.StartsWith("The Bottle is: Into the Switch 1      ,  Position: In the Switch 1"))
        {
            currentPosition = "In_Switch_1";
        }
        else if (payload.StartsWith("The Bottle is: On the Conveyer 2   ,  Position : On the Conveyer 2"))
        {
            currentPosition = "ON_CONVEYER_2";
        }
        else if (payload.StartsWith("The Bottle is: Into the Switch 2      ,  Position: In the Switch 2"))
        {
            currentPosition = "In_Switch_2";
        }
        else if (payload.StartsWith("The Bottle is: On the Conveyer 3   ,  Position : On the Conveyer 3"))
        {
            currentPosition = "ON_CONVEYER_3";
        }
        else if (payload.StartsWith("The Bottle is: Into the Switch 3      ,  Position: In the Switch 3"))
        {
            currentPosition = "In_Switch_3";
        }
        else if (payload.StartsWith("The Bottle is: On the Conveyer 4   ,  Position : On the Conveyer 4"))
        {
            currentPosition = "ON_CONVEYER_4";
        }
        else if (payload.StartsWith("The Bottle is: At the Output      ,  Position:  At the Output"))
        {
            currentPosition = "At_Output";
        }

        lock (lockObj)
        {
            if (currentPosition != null && currentPosition != lastPosition)
            {
                positionMessages.Add(currentPosition);
                lastPosition = currentPosition;
                startTime = DateTime.Now;
            }

            var elapsedTime = (DateTime.Now - startTime).TotalSeconds;

            switch (lastPosition)
            {
                case "ON_CONVEYER_1":
                    x=-0.30094;
                    y=-0.47503;
                    x += 0 * elapsedTime;
                    y += 0.10 * elapsedTime;
                    y = Math.Min(y, -0.090);
                    angle = 0;
                    Console.WriteLine($"ON_CONVEYER_1: x = {x:F2}, y = {y:F3}, angle = {angle:F3}");
                    SendPositionOverTcp(x, y);
                    break;
                case "In_Switch_1":
                    // angle += Math.PI / 2 * (elapsedTime / 2);
                    angle =0;
                    angle += 45 * elapsedTime;
                    angle = Math.Min(angle, 90);
                    x += 7 * Math.Sin(angle);
                    y += 7 - 7 * Math.Cos(angle);
                    angle2 =0;
                    angle3 =0;
                    SendPositionOverTcp(angle,angle2,angle3);
                    //Console.WriteLine($"In_Switch_1: x = {x:F3}, y = {y:F3}, angle = {angle:F3}");
                    break;
                case "ON_CONVEYER_2":
                    x=-0.2202;
                    y=0.01565;
                    x += 0.10 * elapsedTime;
                    y += 0 * elapsedTime;
                    x = Math.Min(x, 0.15883);
                    angle = 0;
                    Console.WriteLine($"ON_CONVEYER2: x = {x:F2}, y = {y:F3}, angle = {angle:F3}");
                    SendPositionOverTcp(x, y);
                    break;
                    break;
                case "In_Switch_2":
                    // angle = -Math.PI / 2;
                    // angle += Math.PI / 2 * (elapsedTime / 2);
                    angle2 = 0;
                    angle2 += 45 * elapsedTime;
                    angle2 = Math.Min(angle2, 180);
                    angle =90;
                    angle3 =0;

                    SendPositionOverTcp(angle,angle2,angle3);
                    // x += 7 * Math.Sin(angle);
                    // y += 7 - 7 * Math.Cos(angle);
                    break;
                case "ON_CONVEYER_3":
                    x=0.36116;
                    y=0.01565;
                    x += 0.10 * elapsedTime;
                    y += 0 * elapsedTime;
                    x = Math.Min(x, 0.70313);
                    SendPositionOverTcp(x, y);

                    break;
                case "In_Switch_3":
                    // angle = -Math.PI / 2;
                    // angle += Math.PI / 2 * (elapsedTime / 2);
                    // x += 7 * Math.Sin(angle);
                    // y += 7 - 7 * Math.Cos(angle);
                    angle3=0;
                    angle3 += 45 * elapsedTime;
                    angle3 = Math.Min(angle3, 90);
                    angle =90;
                    angle2 =0;
                   SendPositionOverTcp(angle,angle2,angle3);
                    break;
                case "ON_CONVEYER_4":
                    x=0.80578;
                    y=-0.073;
                    x += 0 * elapsedTime;
                    y += -0.1 * elapsedTime;
                    y = Math.Max(y, -0.4462);
                    SendPositionOverTcp(x, y);
                    break;
                case "At_Output":
                    x += 0 * elapsedTime;
                    y += 0 * elapsedTime;
                    angle = 0;
                    angle2 =0;
                    angle3 =0;

                    SendPositionOverTcp(x, y);
                    break;
                default:
                    x = 0;
                    y = 0;
                    angle = 0;
                    angle2 =0;
                    angle3 =0;

                    break;
            }

            positionList.Add((timestamp, lastPosition, x, y, angle));
            // SendPositionOverTcp(x, y, z, angle);
        }
    }
    static void SendPositionOverTcp(double x, double y)
    {
        try
        {
            // Ensure that the consumer is connected
            if (m_bConsumerIsConnected && m_consumerStream != null)
            {
                // Create the JSON message
                string message = $"{{\"q\": [{x}, {y}]}}\r\n";

                // Convert the message to bytes
                byte[] data = Encoding.UTF8.GetBytes(message);

                // Send the data over the existing network stream
                m_consumerStream.Write(data, 0, data.Length);

                Console.WriteLine($"Sent: {message}");
            }
            else
            {
                Console.WriteLine("[WARN] Not connected to IsaacSim. Cannot send data.");
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error sending data over TCP: {ex.Message}");
        }
    }
    static void SendPositionOverTcp(double angle1, double angle2, double angle3)
{
    try
    {
        if (m_bConsumerIsConnected && m_consumerStream != null)
        {
            string message = $"{{\"q\": [{angle1}, {angle2}, {angle3}]}}\r\n";
            byte[] data = Encoding.UTF8.GetBytes(message);
            m_consumerStream.Write(data, 0, data.Length);
            Console.WriteLine($"Sent: {message}");
        }
        else
        {
            Console.WriteLine("[WARN] Not connected to IsaacSim. Cannot send data.");
        }
    }
    catch (Exception ex)
    {
        Console.WriteLine($"Error sending data over TCP: {ex.Message}");
    }
}




    static async void UpdateTimer_Elapsed(object sender, ElapsedEventArgs e)
    {
        List<(DateTime timestamp, string position, double x, double y, double angle)> dataCopy;

        lock (lockObj)
        {
            dataCopy = new List<(DateTime timestamp, string position, double x, double y, double angle)>(positionList);
            positionList.Clear();
        }

        if (dataCopy.Count > 0)
        {
            List<JsonPatchDocument> patchList = CreatePatchList(dataCopy);
            await UpdateTwinFromPatchesAsync(patchList, updateTwinIds);
            //await update_twin_from_patches(patchList, updateTwinIds);
        }
    }

    static List<JsonPatchDocument> CreatePatchList(List<(DateTime timestamp, string position, double x, double y, double angle)> positionList)
    {
        List<JsonPatchDocument> patchList = new List<JsonPatchDocument>();

        foreach (var item in positionList)
        {
            JsonPatchDocument patchPosition = new JsonPatchDocument();

            patchPosition.AppendAdd<string>("/value", item.position);
            patchList.Add(patchPosition);

            JsonPatchDocument patchX = new JsonPatchDocument();
            patchX.AppendAdd<string>("/value", item.x.ToString());
            patchList.Add(patchX);

            JsonPatchDocument patchY = new JsonPatchDocument();
            patchY.AppendAdd<string>("/value", item.y.ToString());
            patchList.Add(patchY);

            JsonPatchDocument patchAngle = new JsonPatchDocument();
            patchAngle.AppendAdd<string>("/value", item.angle.ToString());
            patchList.Add(patchAngle);
            //Console.WriteLine("patchList should be:"+patchList);

            //Console.WriteLine("operational_data_patch should be:"+patchPosition); 


           
        }

        return patchList;
    }

    static async Task UpdateTwinFromPatchesAsync(List<JsonPatchDocument> patches, List<string> twinIds)
    {
        Console.WriteLine("Displaying Patches"+patches.ToString);
        foreach (var patch in patches)
        {
            Console.WriteLine(patch.ToString());  //not good: [{"op":"add","path":"/position","value":"ON_CONVEYER_1"},{"op":"add","path":"/x","value":"0"},{"op":"add","path":"/y","value":"10.020056"},{"op":"add","path":"/angle","value":"0"}]

        }

        for (int i = 0; i < patches.Count; i++)
        { Console.WriteLine(patches.Count);
            try
            {
                var twinId = twinIds[i % twinIds.Count];
                await m_azureClient.UpdateDigitalTwinAsync(twinId, patches[i]);
                //m_azureClient.UpdateComponent(twinId,"value", patches[i]);
                Console.WriteLine($"Successfully updated twin {twinId}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error updating twin {twinIds[i % twinIds.Count]}: {ex.Message}");
            }
        }

        //Console.WriteLine("Twin graph updated.");
    }
        static void update_twin_from_patches(List<JsonPatchDocument> patches, List<string> twinIds){
            Console.WriteLine("Displaying Patches");
            Console.WriteLine(patches);
            // update the whole twinID as a whole Sting
            for(int i=0; i<patches.Count; i++){
                m_azureClient.UpdateComponent(twinIds[i], "value", patches[i]); 

            }
            Console.WriteLine("Twin graph updated for Num: " );
        }
}
