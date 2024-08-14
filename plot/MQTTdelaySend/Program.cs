using System;
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
            .WithTcpServer("192.168.80.100", 1883) // 
            .WithCleanSession()
            .Build();

        // Connect to the broker
        await mqttClient.ConnectAsync(options, CancellationToken.None);

        // Send different messages in a loop
        for (int i = 1; i <= 100; i++) // For example, send 10 messages
        {
            string payload = $"number {i}, the bottle is on the conveyor...";
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
            await Task.Delay(100);
        }

        // Disconnect the client
        await mqttClient.DisconnectAsync();
    }

    private static void LogTimestamp(string filename, string message, DateTime timestamp)
    {
        using (var writer = new StreamWriter(filename, true))
        {
            writer.WriteLine($"{message},{timestamp:O}");
        }
    }
}
