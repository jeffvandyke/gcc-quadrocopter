using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Quadrotor_Control
{
    class SensorCollection : List<Sensor>//, IEnumerable<string>
    {

        public IEnumerator<string> getEnumerator()
        {
           List<SensorEnumerator> ls = new List<SensorEnumerator>();

            foreach (Sensor s in this)
            {
               ls.Add(s.getEnumerator());
            }
            return new SensorCollectionEnumerator(ls);
        }
    }

    public class SensorCollectionEnumerator//:IEnumerator<string>
    {
        public SensorCollectionEnumerator(List<SensorEnumerator> ls)
        { }
    }
}
