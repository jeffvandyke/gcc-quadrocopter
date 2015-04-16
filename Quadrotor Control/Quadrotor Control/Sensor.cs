using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Windows.Forms.DataVisualization.Charting;

namespace Quadrotor_Control
{
    public class Sensor: Variable
    {
        private string[] axisNames = {"x-axis", "y-axis", "z-axis"};
        public int axisNum;

        public List<int> Data;

        private string errorText = "Not Available";

        public Sensor(char type, System.Windows.Forms.Label readout)
            : this(type, readout, 3)
        { }

        public Sensor(char type, System.Windows.Forms.Label readout, int varNum)
            : base(type+"", type, readout)
        {

            Data = new List<int>();
            axisNum = varNum;
            for (int i = 0; i < varNum; i++)
            {
                Data.Add(0);
                dataCollection.Add(name + axisNames[i]);
            }
        }

        public void UpdateStateVariables(string newData)
        {
            base.UpdateDisplay(newData);

            char curChar;
            int i = 0;

            for (int j = 0; j < 3; j++)
            {
                Data[j] = 0;
                int X = 0;
                for (curChar=newData[i++]; curChar != ' ' && i<newData.Length; curChar=newData[i++])
                {
                    X *= 10;
                    X += curChar - '0';
                    if (curChar == '-') X = (newData[++i] - '0') * -1;
                }

                UpdateStateVariable(j, X);
                Data[j] = X;
            }

            UpdateDisplay();
        }

        public void UpdateStateVariables(List<int> newValues)
        {
            for (int i=0; i<axisNum; i++)
            {
                UpdateStateVariable(i, newValues[i]);
            }
            Data = newValues;

            UpdateDisplay();
        }

        public void UpdateStateVariable(int key, int x)
        {
            Data[key] = x;
            dataCollection[key].Points.AddY(x);
        }

        public void UpdateDisplay()
        {
            //Output.Text = Data[0] + ", " + Data[1] + ", " + Data[2];
            /*string text = "";
            

            foreach (Int64 i in Data)
            {
                text += ", " + i.ToString();
            }

            if (text.Length == 0) Output.Text = errorText;
            else Output.Text = text.Substring(2);*/
        }

        public SensorEnumerator getEnumerator()
        {
            return new SensorEnumerator(dataCollection);
        }

        public void WriteToFile()
        {

        }

    }
}
