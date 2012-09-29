using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace QuadCopterTool
{
    public class HefnyCopterSerial
    {
        protected byte[] vArray;
        protected System.IO.TextWriter mLogFile;        
      
        protected System.IO.Ports.SerialPort mSerialPort;

        protected string mReceivedData;

        public Int16 GyroY, GyroX, GyroZ, AccY, AccX, AccZ, Volt,Motor1, Motor2, Motor3, Motor4;


        public System.IO.Ports.SerialPort SerialPort
        {
            get
            {
                return mSerialPort;
            }
        }

      public HefnyCopterSerial()
        {
            mSerialPort = new System.IO.Ports.SerialPort();
            mSerialPort.BaudRate = 57600;
            mSerialPort.DataBits = 8;
            mSerialPort.DiscardNull = false;
            mSerialPort.Parity = Parity.None;
            mSerialPort.RtsEnable = true;
            mSerialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(DataReceived);

            vArray = new byte[20];
        }


      protected void CopyData()
      {
          GyroY = BitConverter.ToInt16(vArray, 0);
          GyroZ = BitConverter.ToInt16(vArray, 2);
          GyroX = BitConverter.ToInt16(vArray, 4);
          AccX = BitConverter.ToInt16(vArray, 6);
          AccY = BitConverter.ToInt16(vArray, 8);
          AccZ = BitConverter.ToInt16(vArray, 10);
          //Volt= BitConverter.ToInt16(vArray, 12);
          Motor1 = BitConverter.ToInt16(vArray, 12);
          Motor2 = BitConverter.ToInt16(vArray, 14);
          Motor3 = BitConverter.ToInt16(vArray, 16);
          Motor4 = BitConverter.ToInt16(vArray, 18);
          mLogFile.Write(GyroX);
          mLogFile.Write(",");
          mLogFile.Write(GyroY);
          mLogFile.Write(",");
          mLogFile.Write(GyroZ);
          mLogFile.Write(",");
          mLogFile.Write(AccX);
          mLogFile.Write(",");
          mLogFile.Write(AccY);
          mLogFile.Write(",");
          mLogFile.Write(AccZ);
          mLogFile.Write(",");
          mLogFile.Write(Motor1);
          mLogFile.Write(",");
          mLogFile.Write(Motor2);
          mLogFile.Write(",");
          mLogFile.Write(Motor3);
          mLogFile.Write(",");
          mLogFile.Write(Motor4);
          mLogFile.WriteLine();
      }

      
        public void OpenPort(string PortName,string FileName)
        {
            mLogFile = System.IO.File.CreateText(FileName + System.Environment.TickCount.ToString() + ".csv");
            if (mSerialPort.IsOpen == false)
            {
               // mFile = System.IO.File.CreateText(@"d:\QuadReadings_" + System.Environment.TickCount.ToString() + ".csv");
                mSerialPort.DiscardNull = false;
                mSerialPort.Encoding = Encoding.Unicode;
                mSerialPort.PortName = PortName;
                mSerialPort.Open();
            }
        }

        public void ClosePort()
        {
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
            /* for (j = 0; j < XBEEPort.BytesToRead; ++j)
             {

                 array[j] =(byte) XBEEPort.ReadByte();
             }
            */
           // bStartCopy = false;  // HERE I ignore data if packet ends before E received./// this can be updated to skip packet header and contiue reading.
           // Idx = 0;
            for (int i = 0; i < array.Length; ++i) // j; ++i)
            {
                if ((!bStartCopy) && (array[i] == 'S'))
                {
                    bStartCopy = true;
                    Idx = 0;
                    //i += 1;
                    continue;
                }
                if ((!bStartCopy) && (array[i] == 'E'))
                {
                    if (ExpectEOF == true) // message correct then copy
                    {
                        CopyData();
                       
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

    }
}
