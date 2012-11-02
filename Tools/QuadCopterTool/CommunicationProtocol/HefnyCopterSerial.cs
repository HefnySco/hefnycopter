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
    public class HefnyCopterSerial : HefnyCopterBaseConnection
    {

        private enum ENUM_RxDataType
        {
            Undefined,
            Sensors,
            Settings
        }

        #region "Attributes"

        protected bool ExpectEOF = false;   //if true after receiving data so that we expect EOF flag. if not received then copied data is discarded
        protected bool bStartCopy = false;  //if true data is moved to DataArray
        protected int Idx = 0;              // index of DataArray

        protected byte[] vArray;
        protected ENUM_RxDataType mRxDataType;

        protected System.IO.Ports.SerialPort mSerialPort;
        protected string mPortName;
        protected int mBaudRate;

        #endregion


        #region "Properties"

        public string FileName
        {
            get;
            set;
        }

        public System.IO.Ports.SerialPort SerialPort
        {
            get
            {
                return mSerialPort;
            }
        }



        public string PortName
        {
            get
            {
                return mPortName;
            }
            set
            {
                if (mSerialPort.IsOpen == true)
                {
                    throw new InvalidOperationException("Port is Open");
                }

                mPortName = value;
            }
        }

        public int BaudRate
        {
            get
            {
                return mBaudRate;

            }
            set
            {
                if (mSerialPort.IsOpen == true)
                {
                    throw new InvalidOperationException("Port is Open");
                }
                mBaudRate = value;
            }
        }

        #endregion

        public HefnyCopterSerial()
        {
            mRxDataType = ENUM_RxDataType.Undefined;
            mSerialPort = new System.IO.Ports.SerialPort();
            mSerialPort.DataBits = 8;
            mSerialPort.DiscardNull = false;
            mSerialPort.Parity = Parity.None;
            mSerialPort.RtsEnable = true;
            mSerialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(DataReceived);

            vArray = new byte[20];

        }






        protected override void CopyData(byte[] vArray)
        {
            mdelegate_CopyData(vArray);
        }




        public override void Open()
        {

            if (mSerialPort.IsOpen == false)
            {
                mSerialPort.DiscardNull = false;
                mSerialPort.Encoding = Encoding.Unicode;
                mSerialPort.PortName = mPortName;
                mSerialPort.BaudRate = mBaudRate;
                mSerialPort.Open();
            }
        }






        public override void Close()
        {
            if (mSerialPort.IsOpen == true)
            {
                mSerialPort.Close();
            }
        }




        /// <summary>
        /// Sensor Protocol:
        /// S,GX,GY,GZ,AccX,AccY,AccZ,Bat,M1,M2,M3,M4,E
        /// Settings Protocol:
        /// C,.......................................,E
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            string RxString;
            RxString = (mSerialPort.ReadExisting());

            byte[] array = new byte[8000]; //

            array = Encoding.Unicode.GetBytes(RxString);
          

            for (int i = 0; i < array.Length; ++i) // j; ++i)
            {
                if ((!bStartCopy) && (array[i] == 'S'))
                {
                    mRxDataType = ENUM_RxDataType.Sensors;
                    bStartCopy = true;
                    Idx = 0;
                    continue;
                }
                if ((!bStartCopy) && (array[i] == 'C'))
                {
                    mRxDataType = ENUM_RxDataType.Settings;
                    bStartCopy = true;
                    Idx = 0;
                    continue;
                }
                if ((!bStartCopy) && (array[i] == 'E'))
                {
                    mRxDataType = ENUM_RxDataType.Undefined;
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



    }
}
