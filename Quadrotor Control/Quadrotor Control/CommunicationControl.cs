using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Xna.Framework;
using System.Threading;

namespace Quadrotor_Control
{
    class CommunicationControl : Bluetooth
    {
        private bool quadOpen = false;

        public Vector3 pos, rot, vel;

        public CommunicationControl() : base()
        {
        }

        public void InitializeQuad()
        {
            StreamReader sr = new StreamReader("init.txt");
            WriteInt((short)(Int16.Parse(sr.ReadLine()) << 3));
            for (int i = 0; i < 5; i++)
            {
                WriteFloat(Single.Parse(sr.ReadLine()));
            }
        }


        /* needs to be called in a separate thread
         * type 1 for setpoint
         * type 2 for gps
         */
        public bool SendData(short type, short x, short y)
        {
            if (!quadOpen) return false;
            WriteInt(type);
            WriteInt(x);
            WriteInt(y);

            quadOpen = false;
            return true;
        }

        public void ReadState()
        {
                pos.X = ReadFloat();
                pos.Y = ReadFloat();
                pos.Z = ReadFloat();
                rot.X = ReadFloat();
                rot.Y = ReadFloat();
                rot.Z = ReadFloat();
        }

        private void ParseCommand(int command)
        {
            if (command != 0) quadOpen = true;
            switch (command)
            {
                case 1:
                    ReadState();
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 7:
                    InitializeQuad();
                    break;
                default: break;
            }
        }

        public void OpenPort(String commPort)
        {
            // connect via port selected
            PortName = commPort;

            // open connection to quadrotor bluetooth
            Open();
            ReadTimeout = 1000;
            WriteTimeout = 1000;
            // connect to device
            WriteLine("C,0006666007BA");
            

            // change color of connection status

            // start reading
            //DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

            //new Thread(InitializeQuad);
        }

        public void DataReceivedHandler(object o, SerialDataReceivedEventArgs e)
        {
            //ParseCommand(ReadByte());
        }



    }
}
