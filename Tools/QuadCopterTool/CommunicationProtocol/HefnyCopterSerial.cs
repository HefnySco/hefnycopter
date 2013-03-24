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

       
        protected const byte CONST_FIRMWARE_SIGNATURE = 0xfe;

        #region "Attributes"

        protected bool ExpectEOF = false;   //if true after receiving data so that we expect EOF flag. if not received then copied data is discarded
        protected bool bStartCopy = false;  //if true data is moved to DataArray
        protected int Idx = 0;              // index of DataArray

        protected byte[] vArray,cArray;
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
            cArray = new byte[400];

        }






        protected override void CopyData(ENUM_RxDataType DataType, byte[] DataArray)
        {
            mdelegate_CopyData(DataType, DataArray);
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




        public override void SendCommand(HefnyCopterCommand HefnyCopterCommand)
        {
            byte[] Command = HefnyCopterCommand.CommandBytes;

            if (Command.Length != 9) return;
            mSerialPort.Write(Command,0,Command.Length);
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
                // Sensors Start Tag .... Data of Gyros & Acc readinsg while flying is being recived.
                if ((!bStartCopy) && (array[i] == 'S'))
                {
                    mRxDataType = ENUM_RxDataType.Sensors;
                    bStartCopy = true;
                    Idx = 0;
                    continue;
                }
                // Configuration Start Tag .... Data of Config structure is being received
                if ((!bStartCopy) && (array[i] == 'C'))
                {
                    mRxDataType = ENUM_RxDataType.Settings;
                    bStartCopy = true;
                    Idx = 0;
                    continue;
                }
                // End Tag
                if ((!bStartCopy) && (array[i] == 'E'))
                {
                    if (ExpectEOF == true) // message correct then copy
                    {switch (mRxDataType)
                    {
                        case ENUM_RxDataType.Sensors:
                            CopyData(mRxDataType,vArray);
                            break;
                        case ENUM_RxDataType.Settings:
                            CopyData(mRxDataType,cArray);
                            break;

                    }
                    }
                    mRxDataType = ENUM_RxDataType.Undefined;
                    
                        

                    
                    ExpectEOF = false;
                    continue;
                }
                // Incoming Data Copying
                if (bStartCopy)
                {
                    switch (mRxDataType)
                    {
                        case ENUM_RxDataType.Sensors:
                            if (Idx == 20)
                            {
                                Idx = 0;
                                bStartCopy = false;
                                ExpectEOF = true;
                                i -= 1; // stepback to check for 'E'
                                continue;
                            }
                            vArray[Idx] = array[i];
                            break;

                        case ENUM_RxDataType.Settings:
                            if (Idx == 0)
                            {
                                // verify data is OK
                                if (array[i] == CONST_FIRMWARE_SIGNATURE)
                                { // signature is OK 
                                }
                            }

                            if (Idx == 86)
                            {
                                Idx = 0;
                                bStartCopy = false;
                                ExpectEOF = true;
                                i -= 1; // stepback to check for 'E'
                                continue;
                            }
                            cArray[Idx] = array[i];
                            break;
                   }
                    Idx += 1;
                }
               ExpectEOF = false;
            }
        //   Idx += 1; // FIX bug when DataMisalignedException comes CONST_FIRMWARE_SIGNATURE two chunks cArray was shifted by one.
        }
    }
}
