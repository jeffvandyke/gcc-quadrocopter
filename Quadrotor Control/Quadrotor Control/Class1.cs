using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Quadrotor_Control
{
    public class Sensor
    {
        string type{ get; }

        public Sensor(string type)
        {
            this.type = type;
        }

        public void updateStateVariables()
        {
        }
        

    }
}
