using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace Quadrotor_Control
{
   class Bluetooth
   {
      private SerialPort connection;
      private Thread readThread;
      private Action<string> callbackFunction;

      public Bluetooth(string portName, int baudRate, Parity parity, int dataBits, StopBits stopBits)
      {
         connection = new SerialPort(portName, baudRate, parity, dataBits, stopBits);
      }      

      public void OpenConnection(int readTimeout, int writeTimeout, string address)
      {
         this.readThread = new Thread(new ThreadStart(this.Read));
         connection.Open();
         connection.ReadTimeout  = readTimeout;
         connection.WriteTimeout = writeTimeout;
         // connect to device
         connection.WriteLine("C," + address);
      }

      public void SetPort(string port)
      {
         connection.PortName = port;
      }

      public void CloseConnection()
      {
         connection.Close();
      }

      public bool IsConnected()
      {
         // return connection status
         return connection.IsOpen;
      }

      public void Transmit(string message)
      {
         connection.WriteLine(message);
      }

      public void setupReadCallback(Action<string> callback)
      {
         this.callbackFunction = callback;
      }

      public void StartReading()
      {
         this.readThread.Start();
      }

      public void StopReading()
      {
         this.readThread.Abort();
      }

      public void Read()
      {
         string data;
         while (true)
         {
            // wait 1ms to not hog CPU
            Thread.Sleep(1);
            
            try
            {
               data = connection.ReadLine();
               this.callbackFunction(data);
            }
            catch (TimeoutException ex)
            {
               // do nothing
            }
            catch (Exception ex)
            {
               // don't send the exception since it would mess with the log files
               //this.callbackFunction(String.Format("{0}: {1}", ex.GetType().Name, ex.Message));
            }
         }
      }
   }
}
