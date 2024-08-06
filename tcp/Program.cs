using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace TcpReceiver
{
    class Program
    {
        private const int Port = 8080; // 
        private const int BufferSize = 4096; // 
        private static int messageCount = 0; // 

        static async Task Main(string[] args)
        {
            TcpListener server = null;

            try
            {
                // 
                IPAddress localAddr = IPAddress.Any;
                server = new TcpListener(localAddr, Port);

                // 
                server.Start();
                Console.WriteLine("Waiting for a connection...");

                // 
                TcpClient client = await server.AcceptTcpClientAsync();
                Console.WriteLine("Client connected.");

                // 
                NetworkStream stream = client.GetStream();
                byte[] buffer = new byte[BufferSize]; //
                StringBuilder messageBuilder = new StringBuilder();

                int bytesRead;

                // 
                while ((bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length)) > 0)
                {
                    // 
                    string data = Encoding.ASCII.GetString(buffer, 0, bytesRead);
                    messageBuilder.Append(data);

                    // 
                    ProcessReceivedData(messageBuilder.ToString());
                }

                // 
                Console.WriteLine($"Total messages received: {messageCount}");

                // 
                client.Close();
            }
            catch (Exception e)
            {
                Console.WriteLine($"Exception: {e.Message}");
            }
            finally
            {
                // 
                server?.Stop();
            }
        }

        // 
        private static void ProcessReceivedData(string data)
        {
            // 
            string[] messages = data.Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);

            foreach (var message in messages)
            {
                // 
                messageCount++;

                // 
                Console.WriteLine($"Received message #{messageCount}: {message}");
            }
        }
    }
}
