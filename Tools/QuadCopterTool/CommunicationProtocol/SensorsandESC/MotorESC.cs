using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol.Sensors
{
    public class MotorESC: ValueBase
    {

        public MotorESC()
        {
            mUpperLimit = 1000;
            mLowerLimit = 0;
        }
    }
}
