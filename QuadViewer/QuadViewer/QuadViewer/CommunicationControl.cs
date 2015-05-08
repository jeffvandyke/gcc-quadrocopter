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
                WriteInt(Int16.Parse(sr.ReadLine()));
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
            pos.Z = 0; ReadFloat();
            pos.Y = 0; ReadFloat();
            pos.X = 0; ReadFloat();

                float newZ = ReadFloat();
                float newX = ReadFloat();
                float newY = ReadFloat();

                //if (Math.Abs((newX - rot.X) % 360) < 45)
                    rot.X = newX;
                //if (Math.Abs((newY - rot.Y) % 360) < 45)
                    rot.Y = newY;
                //if (Math.Abs((newZ - rot.Z) % 360) < 45)
                    rot.Z = newZ;

                rot.X *= ((float)Math.PI / 180);
                rot.Y *= ((float)Math.PI / 180);
                rot.Z *= ((float)Math.PI / 180);
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
                    //InitializeQuad();
                    break;
                default: break;
            }
        }

        public void OpenPort(String commPort, SerialDataReceivedEventHandler handler)
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
            DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
            DataReceived += handler;

            new Thread(InitializeQuad);
        }

        public void DataReceivedHandler(object o, SerialDataReceivedEventArgs e)
        {
            try
            {
                ParseCommand(ReadByte());
                ReadExisting();
            }
            catch (Exception ex)
            { }
        }



    }
}
