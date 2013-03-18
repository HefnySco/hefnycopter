using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol
{
    public class HefnyCopterCommand
    {
        protected const int CRC_BYTE =8;

        protected byte[] mCommandArray = new byte[9];

        #region "Properties"

        public byte[] CommandBytes
        {
            get
            {
                CalculateCheckSum();
                return mCommandArray;
            }
        }

        #endregion 


        #region "Construction"

        /// <summary>
        /// Send command to quadcopter.
        /// <para>
        /// <list>
        /// Protocol:
        /// #CMD#: 1 byte
        /// 'C'		: SETCMD
        ///			CMD:		1 byte
        ///						0	--		DISARM	
        ///						1   --		BLINK
        ///						2	--		X-MODE
        ///						3	--		P-MODE
        ///						4	--		CALIBRATE_ACC
        ///						
        ///			DataValue:	6
        ///			CHKSUM:		1 byte
        ///			</list>
        /// </para>
        /// </summary>
        /// <param name="CMD_ID"></param>
        public HefnyCopterCommand(ENUM_SerialCommands CMD_ID)
        {
            mCommandArray[0] = (byte) 'C';  // Execute Action Command
            mCommandArray[1] = (byte)CMD_ID;
        }


        /// <summary>
        /// Set Value in Config structure in Quadcopter.
        /// this includes changing flying mode, PID settings ...etc.
        /// <para>
        /// 'V'		: SETVALUE:   0 byte
        /// 		DataOffset:   2 bytes		[0-65536]
        /// 		DataLength:   1 byte		[0,1,2,3,4]
        ///			DataValue:    4 bytes		[Word-32]
        ///			CHKSUM:		  1 byte
        /// </para>
        /// </summary>
        /// <param name="Parameter"></param>
        /// <param name="DataBytes"></param>
        public HefnyCopterCommand(ENUM_Parameter Parameter, byte DataLength, Int32 DataValue)
        {
            byte[] param;
            mCommandArray[0] = (byte) 'V';  // Set Value Command
            param = BitConverter.GetBytes((Int16)Parameter);
            mCommandArray[1] = param[0];
            mCommandArray[2] = param[1];

            mCommandArray[3] = DataLength;
            
            param = BitConverter.GetBytes((Int32)DataValue);
            mCommandArray[4] = param[0];
            mCommandArray[5] = param[1];
            mCommandArray[6] = param[2];
            mCommandArray[7] = param[3];

        }

        #endregion



        #region "Methods"

        protected void CalculateCheckSum()
        {
            byte CRC = 0;
            for (int i = 0; i < 8; ++i)
            {
                CRC += (byte)mCommandArray[i];
            }
            mCommandArray[CRC_BYTE] = CRC;
        }

        #endregion 


    }
}
