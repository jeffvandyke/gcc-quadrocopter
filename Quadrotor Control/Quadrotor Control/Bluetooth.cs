using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Quadrotor_Control
{
    public class Bluetooth : SerialPort
    {
        public bool closing { get; set; }

        public Bluetooth()
            : base("COM", 115200, Parity.None, 8, StopBits.One)
        { }

        public Bluetooth(string portName, int baudRate, Parity parity, int dataBits, StopBits stopBits) :
            base(portName, baudRate, parity, dataBits, stopBits)
        { }

        public void WriteInt(Int16 data)
        {
            this.Write(BitConverter.GetBytes(data), 0, 2);
        }

        public void WriteFloat(Single data)
        {
            this.Write(BitConverter.GetBytes(data), 0, 4);
        }

        public short ReadShort()
        {
            return (short)(ReadByte() | (ReadByte()<<8));
        }

        public int ReadInt()
        {
            return (int)(ReadByte() | ReadByte() << 8 | ReadByte() << 16 | ReadByte() << 24);
        }

        public float ReadFloat()
        {
            //return (float)(ReadByte() | ReadByte() << 8 | ReadByte() << 16 | ReadByte() << 24);
            return BitConverter.ToSingle(
                BitConverter.GetBytes(ReadByte() | ReadByte() << 8 | ReadByte() << 16 | ReadByte() << 24), 0);
        }

        public int ReadByte()
        {
            try
            {
                return base.ReadByte();
            }
            catch (IOException ex)
            { }
            return 0;
        }

    

    }
}
