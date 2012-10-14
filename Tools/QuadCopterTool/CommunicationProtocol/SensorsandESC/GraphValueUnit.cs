using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol.Sensors
{
    public class GraphValueUnit
    {

        public Int32 X
        { get; set; }

        public Int16 Y
        { get; set; }


        public GraphValueUnit(Int32 ValueX, Int16 ValueY)
        {
            this.X = ValueX;
            this.Y= ValueY;
        }

    }
}
