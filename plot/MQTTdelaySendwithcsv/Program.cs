using System;
using System.Globalization;
using System.IO;
using System.Threading.Tasks;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;

class Program
{
    private static async Task Main(string[] args)
    {
        // Create MQTT client
        var factory = new MqttFactory();
        var mqttClient = factory.CreateMqttClient();

        // Configure client options
        var options = new MqttClientOptionsBuilder()
            .WithClientId("CX-2040-Sender")
            .WithTcpServer("192.168.80.100", 1883)
            .WithCleanSession()
            .Build();

        // Connect to the broker
        await mqttClient.ConnectAsync(options, CancellationToken.None);

        // Send different messages in a loop
        for (int i = 1; i <= 2000; i++) // For example, send 100 messages
        {
            string payload = $"number {i}, the bottle is on the conveyor 1";
            var timestamp = DateTime.UtcNow;

            // Construct MQTT message
            var message = new MqttApplicationMessageBuilder()
                .WithTopic("MyJoghurt2Panda/ProvideBottle")
                .WithPayload(payload)
                .WithExactlyOnceQoS()
                .WithRetainFlag(false)
                .Build();

            // Publish the message
            await mqttClient.PublishAsync(message, CancellationToken.None);

            // Log the timestamp to a CSV file
            LogTimestamp("publisher.csv", payload, timestamp);

            // Wait for a while before sending the next message (e.g., 1 second)
            await Task.Delay(100); // Changed delay to 1 second
        }

        // Disconnect the client
        await mqttClient.DisconnectAsync();
    }

    private static void LogTimestamp(string filename, string message, DateTime timestamp)
    {
        // Ensure file exists and write headers if necessary
        if (!File.Exists(filename))
        {
            using (var writer = new StreamWriter(filename, false))
            {
                writer.WriteLine("message,timestamp");
            }
        }

        using (var writer = new StreamWriter(filename, true))
        {
            // Log with high precision timestamp
            writer.WriteLine($"{EscapeCsvValue(message)},{timestamp.ToString("yyyy-MM-dd HH:mm:ss.ffffff", CultureInfo.InvariantCulture)}");
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
