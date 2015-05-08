using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.DataVisualization.Charting;

namespace Quadrotor_Control
{
    public class Variable
    {
        protected string name;
        public char type { get; private set; }

        private System.Windows.Forms.Label Output;

        int currentData;
        public List<Series> dataCollection;
        int average;

        public Variable(string name, char type, System.Windows.Forms.Label readout)
        {
            dataCollection = new List<Series>();
            this.name = name;
            Output = readout;
            this.type = type;
            average = 0;
        }

        public void UpdateStateVariables(string newData)
        {
            UpdateDisplay(newData);


        }

        public void UpdateDisplay(string newData)
        {
            Output.Text = name+": "+newData;

        }

        public void ToggleRecording(bool on)
        { }

    }
}
