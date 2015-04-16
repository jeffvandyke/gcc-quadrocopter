using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.DataVisualization.Charting;

namespace Quadrotor_Control
{
    public class SensorEnumerator
    {
        List<IEnumerator<DataPoint>> dataEnums;

        public SensorEnumerator(List<IEnumerator<DataPoint>> startValues)
        {
            dataEnums = startValues;
        }

        public SensorEnumerator(SeriesCollection collection)
        {
            foreach (Series s in collection)
            {
                dataEnums.Add(s.Points.GetEnumerator());
            }
        }

        public string Current {
            get
            {
                string result = "";
                foreach (IEnumerator<DataPoint> ienum in dataEnums)
                {
                    result = ienum.Current+",";
                }
                return result.Substring(0, result.Count() - 1);
            } 
       }

        public bool MoveNext()
        {
            bool result = true;
            foreach (IEnumerator<DataPoint> ienum in dataEnums)
            {
                result = result && ienum.MoveNext();
            }
            return result;
        }

        public void Reset()
        {
            foreach (IEnumerator<DataPoint> ienum in dataEnums)
            {
                ienum.Reset();
            }
        }
    }
}
