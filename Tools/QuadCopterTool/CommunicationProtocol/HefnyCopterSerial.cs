using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.IO;
using System.Windows.Threading;

using HefnyCopter.CommunicationProtocol.Sensors;

namespace HefnyCopter.CommunicationProtocol
{
    public class HefnyCopterSerial
    {

        protected System.Windows.Threading.DispatcherTimer mTimer;

        public delegate void delegate_CopyData (byte[] vArray);
        public delegate_CopyData mdelegate_CopyData;

        protected byte[] vArray;
        protected System.IO.TextWriter mLogFile;

        protected System.IO.Ports.SerialPort mSerialPort;

        protected string mReceivedData;




        public System.IO.Ports.SerialPort SerialPort
        {
            get
            {
                return mSerialPort;
            }
        }

        public HefnyCopterSerial()
        {

            mTimer = new DispatcherTimer();
            mTimer.Interval = TimeSpan.FromMilliseconds(100);
            mTimer.Tick += new EventHandler(FakeDataReceived);
            

            mSerialPort = new System.IO.Ports.SerialPort();
            mSerialPort.DataBits = 8;
            mSerialPort.DiscardNull = false;
            mSerialPort.Parity = Parity.None;
            mSerialPort.RtsEnable = true;
            mSerialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(DataReceived);

            vArray = new byte[20];

        }

        protected void LogHeader()
        {
            mLogFile.Write("Time");
            mLogFile.Write(","); 
            mLogFile.Write("Gyro_X");
            mLogFile.Write(",");
            mLogFile.Write("Gyro_Y");
            mLogFile.Write(",");
            mLogFile.Write("Gyro_Z");
            mLogFile.Write(",");
            mLogFile.Write("Acc_X");
            mLogFile.Write(",");
            mLogFile.Write("Acc_Y");
            mLogFile.Write(",");
            mLogFile.Write("Acc_Z");
            mLogFile.Write(",");
            mLogFile.Write("Motor1");
            mLogFile.Write(",");
            mLogFile.Write("Motor2");
            mLogFile.Write(",");
            mLogFile.Write("Motor3");
            mLogFile.Write(",");
            mLogFile.Write("Motor4");
            mLogFile.WriteLine();
        }


        protected void LogData()
        {
            mLogFile.Write(Environment.TickCount);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Gyro_X.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Gyro_Y.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Gyro_Z.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Acc_X.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Acc_Y.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Acc_Z.LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Motors[0].LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Motors[1].LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Motors[2].LastValue);
            mLogFile.Write(",");
            mLogFile.Write(SensorManager.Motors[3].LastValue);
            mLogFile.WriteLine();
        }


        /// <summary>
        /// used to simulate inpute measures received by the quad
        /// </summary>
        /// <param name="vArray"></param>
        public void SimulateInputResult(byte[] vArray)
        {
            CopyData(vArray);
        }

        protected void CopyData(byte[] vArray)
        {

           
      

            mdelegate_CopyData(vArray);
             LogData(); 
       }


        public void OpenPortFake(string FileName)
        {
            
            mLogFile = System.IO.File.CreateText(FileName);
            LogHeader();
            mTimer.Start();
            
        }

        public void OpenPort(string PortName, int BaudRate, string FileName)
        {
            mLogFile = System.IO.File.CreateText(FileName);
            LogHeader();
            if (mSerialPort.IsOpen == false)
            {
                mSerialPort.DiscardNull = false;
                mSerialPort.Encoding = Encoding.Unicode;
                mSerialPort.PortName = PortName;
                mSerialPort.BaudRate = BaudRate;
                mSerialPort.Open();
            }
        }


        public void ClosePortFake()
        {
            mTimer.Stop();
            mLogFile.Close();
        }


        public void ClosePort()
        {
            mTimer.Stop();
            mLogFile.Close();
            if (mSerialPort.IsOpen == true)
            {
                mSerialPort.Close();
            }
        }


        bool ExpectEOF = false;   //if true after receiving data so that we expect EOF flag. if not received then copied data is discarded
        bool bStartCopy = false;  //if true data is moved to DataArray
        int Idx = 0;              // index of DataArray

        private void DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            string S;
            //SB.Clear();
            S = (mSerialPort.ReadExisting());

            byte[] array = new byte[8000]; //

            array = Encoding.Unicode.GetBytes(S);
            int j;
          
            for (int i = 0; i < array.Length; ++i) // j; ++i)
            {
                if ((!bStartCopy) && (array[i] == 'S'))
                {
                    bStartCopy = true;
                    Idx = 0;
                    continue;
                }
                if ((!bStartCopy) && (array[i] == 'E'))
                {
                    if (ExpectEOF == true) // message correct then copy
                    {
                        CopyData(vArray);

                    }
                    ExpectEOF = false;
                    continue;
                }
                if (bStartCopy)
                {
                    if (Idx == 20)
                    {
                        Idx = 0;
                        bStartCopy = false;
                        ExpectEOF = true;
                        i -= 1; // stepback to check for 'E'
                        continue;
                    }
                    vArray[Idx] = array[i];
                    Idx += 1;
                }

                ExpectEOF = false;
            }


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
    }
}
