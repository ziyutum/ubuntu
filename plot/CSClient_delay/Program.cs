using System;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Timers;
using System.Collections.Generic;

namespace TCPDataReceiver
{
    class Program
    {
        static string m_deviceIP = "127.0.0.1";
        static int m_devicePort = 8080;
        static TcpClient m_deviceClient;
        static NetworkStream m_deviceClientStream;
        static bool m_bConnectedToDevice;

        static int m_numValidData = 0;
        static List<string> m_dataTimestamps = new List<string>();

        static void Main(string[] args)
        {
            Console.WriteLine("Starting TCP Data Receiver...");

            initialize_device_connection();

            while (!m_bConnectedToDevice) 
            { 
                System.Threading.Thread.Sleep(1000);
            }

            process_and_record_data();

            Console.WriteLine("Press Enter to end program...");
            Console.ReadLine();
        }

        static void initialize_device_connection()
        {
            m_bConnectedToDevice = false;
            try_device_connection();
        }

        static void try_device_connection()
        {
            Console.WriteLine($"[INFO] Trying to connect to device with IP: {m_deviceIP}");

            try
            {
                m_deviceClient = new TcpClient(m_deviceIP, m_devicePort);
                m_deviceClientStream = m_deviceClient.GetStream();
                m_bConnectedToDevice = true;
                Console.WriteLine("[SUCCESS] Connected with device.");
            }
            catch (SocketException)
            {
                Console.WriteLine("[WARN] Socket error occurred.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[WARN] Error in connection establishment occurred: {ex.Message}");
            }
        }

        static void process_and_record_data()
        {
            Console.WriteLine("[INFO] Begin reading and recording of device stream.");

            byte[] buffer = new byte[1024];
            int bytesRead;

            Timer timerMaxRead = new Timer(10000); // Stop after 50 seconds
            timerMaxRead.Elapsed += breakReadLoop;
            timerMaxRead.Enabled = true;

            bool continueReading = true;

            void breakReadLoop(Object source, ElapsedEventArgs e)
            {
                Console.WriteLine("Breaking Read Loop");
                continueReading = false;
                timerMaxRead.Stop();
                timerMaxRead.Dispose();

                // Write timestamps to CSV
                string dataFilePath = "data_export/timestamps.csv";
                using (StreamWriter writer = new StreamWriter(dataFilePath))
                {
                    writer.WriteLine("timestamp");
                    foreach (string timestamp in m_dataTimestamps)
                    {
                        writer.WriteLine(timestamp);
                    }
                }
                Console.WriteLine($"[SUCCESS] Exported recorded timestamps to {dataFilePath} (Datapoints: {m_dataTimestamps.Count})");
            }

            while (continueReading)
            {
                try
                {
                    if ((bytesRead = m_deviceClientStream.Read(buffer, 0, buffer.Length)) != 0)
                    {
                        string ts_received = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
                        m_dataTimestamps.Add(ts_received);
                        Console.WriteLine($"Data received at: {ts_received}");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[ERROR] Error while reading data: {ex.Message}");
                }
            }
        }
    }
}
