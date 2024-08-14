using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading.Tasks;

class Program
{
    private static string lastReceivedMessage = null;
    private static string csvFilePath = "messages.csv";

    static async Task Main(string[] args)
    {
        var factory = new MqttFactory();
        var mqttClient = factory.CreateMqttClient();

        var options = new MqttClientOptionsBuilder()
            .WithClientId("CX-2040-Receiver")
            .WithTcpServer("192.168.80.100", 1883)
            .WithCleanSession()
            .Build();

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
            var topic = e.ApplicationMessage.Topic;
            var payload = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
            var timestamp = DateTime.UtcNow;  // Use UTC for consistent timestamping

            var currentMessage = $"[{timestamp:yyyy-MM-dd HH:mm:ss.ffffff}] Message received on topic {topic}: {payload}";

            if (currentMessage != lastReceivedMessage)
            {
                Console.WriteLine(currentMessage);
                lastReceivedMessage = currentMessage;

                // Write to CSV
                WriteToCsv(timestamp, topic, payload);
            }
        });

        try
        {
            await mqttClient.ConnectAsync(options, CancellationToken.None);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }

        // Keep the application running
        Console.WriteLine("Press any key to exit.");
        Console.ReadLine();

        await mqttClient.DisconnectAsync();
    }

    private static void WriteToCsv(DateTime timestamp, string topic, string payload)
    {
        // Ensure the file exists and has headers
        if (!File.Exists(csvFilePath))
        {
            using (var sw = new StreamWriter(csvFilePath))
            {
                sw.WriteLine("timereceive,topic,payload");
            }
        }

        // Append new data
        using (var sw = new StreamWriter(csvFilePath, append: true))
        {
            var csvLine = $"{timestamp.ToString("yyyy-MM-dd HH:mm:ss.ffffff", CultureInfo.InvariantCulture)},{EscapeCsvValue(topic)},{EscapeCsvValue(payload)}";
            sw.WriteLine(csvLine);
        }
    }

    private static string EscapeCsvValue(string value)
    {
        if (value.Contains("\"") || value.Contains(",") || value.Contains("\n"))
        {
            value = "\"" + value.Replace("\"", "\"\"") + "\"";
        }
        return value;
    }
}
