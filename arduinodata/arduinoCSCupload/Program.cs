using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text.Json;
using System.Threading.Tasks;
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using System.IO;

class Program
{
    private static SerialPort serialPort;
    private static DigitalTwinsClient m_azureClient;
    private static readonly List<string> updateTwinIds = new List<string> { "Product_IsEmpty" };

    static async Task Main(string[] args)
    {
        string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";

        // Authenticate with the Azure service
        var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
        m_azureClient = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
        Console.WriteLine("Service client successfully created.");

        string portName = "/dev/ttyACM0";
        int baudRate = 9600;

        serialPort = new SerialPort(portName, baudRate);
        serialPort.DataReceived += DataReceivedHandler;

        try
        {
            serialPort.Open();
            Console.WriteLine("Listening on port " + portName + "...");
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }
        catch (UnauthorizedAccessException ex)
        {
            Console.WriteLine("Access denied to the port: " + ex.Message);
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error: " + ex.Message);
        }
        finally
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
    }

    private static async void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
    {
        SerialPort sp = (SerialPort)sender;
        try
        {
            while (sp.BytesToRead > 0)
            {
                string data = sp.ReadLine().Trim();
                Console.WriteLine("Received: " + data);

                string twinId = updateTwinIds[0];
                var patch = new JsonPatchDocument();
                patch.AppendAdd("/value", data);

                try
                {
                    await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);
                    Console.WriteLine("Twin updated successfully.");
                }
                catch (RequestFailedException ex)
                {
                    Console.WriteLine($"Azure Digital Twins Error: {ex.Message}");
                }
            }
        }
        catch (TimeoutException)
        {
            Console.WriteLine("Timeout error while reading data.");
        }
        catch (IOException ex)
        {
            Console.WriteLine("I/O Error: " + ex.Message);
        }
        catch (InvalidOperationException ex)
        {
            Console.WriteLine("Invalid Operation: " + ex.Message);
        }
    }
}
