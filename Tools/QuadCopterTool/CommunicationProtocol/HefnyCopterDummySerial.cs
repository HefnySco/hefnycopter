using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Threading;
using HefnyCopter.CommunicationProtocol.Sensors;


namespace HefnyCopter.CommunicationProtocol
{
    public class HefnyCopterDummySerial: HefnyCopterSerial
    {

        #region "Attributes"

        protected System.Windows.Threading.DispatcherTimer mTimer;
     

        #endregion 



        #region "Properties"


      
        #endregion 



        #region "Constructors"

        public HefnyCopterDummySerial()
        {
            mTimer = new DispatcherTimer();
            mTimer.Interval = TimeSpan.FromMilliseconds(100);
            mTimer.Tick += new EventHandler(FakeDataReceived);

        }

        #endregion 


        #region "Methods"

        public override void Open()
        {
            mTimer.Start();
        }

        protected override void CopyData(byte[] vArray)
        {
            mdelegate_CopyData(vArray);
        }


        private void FakeDataReceived(object sender, EventArgs e)
        {
            #region "Dummy Data"

            for (int i = 0; i < 20; ++i)
            {
                vArray[i] = (byte)(System.Environment.TickCount / (i + 1));
            }
            CopyData(vArray);

            #endregion
        }

        public override void Close()
        {
            mTimer.Stop();
        }
        #endregion 

    }
}
