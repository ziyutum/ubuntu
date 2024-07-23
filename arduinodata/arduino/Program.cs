using System;
using System.IO;
using System.IO.Ports;
using System.Threading;

class Program
{
    private static SerialPort serialPort;

    static void Main(string[] args)
    {
        string portName = "/dev/ttyACM0"; // 
        int baudRate = 9600; // 

        serialPort = new SerialPort(portName, baudRate);
        serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

        try
        {
            serialPort.Open();
            // 
            Thread.Sleep(2000);
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

    private static void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
    {
        SerialPort sp = (SerialPort)sender;
        try
        {
            while (sp.BytesToRead > 0) // 
            {
                string data = sp.ReadLine().Trim(); // 
                Console.WriteLine("Received: " + data);
            }
        }
        catch (TimeoutException)
        {
            
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
