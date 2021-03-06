﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;




namespace HefnyCopter.CommunicationProtocol
{

     

    public enum ENUM_RxDataType
    {
        Undefined,
        IMU,
        Sensors,
        Settings
    }

    public enum ENUM_SerialCommands
    {
        LED_BLINK     =0x1, // blink LED usually to know that connection is active
        PID_CONFIG   = 0x2, // read PID configs of GYRO, ACC, SONAR
        READ_CONFIG   =0x5, // order vehicle to send config data. [obscolete]
        SAVE_CONFIG   =0x6, // save to EPROM
        CALIBRATE_ACC =0x7,
    }

    /// <summary>
    /// Value offset for each parameter in GlobalValues.h in HefnyCopter2
    /// </summary>
    /// 
    public enum ENUM_Parameter 
    {
        AUTO_DISARM         = 5,
        BOARD_ORIENTATION   = 8,
        FLYING_MODE         = 9,
        THROTTLE_MIN        = 11, 
        STICK_SCALING       = 12,
        
        GYRO_PARAMS_PITCH_P         = 18,
        GYRO_PARAMS_PITCH_P_LIM     = 20,
        GYRO_PARAMS_PITCH_I         = 22,
        GYRO_PARAMS_PITCH_I_LIM     = 24,
        GYRO_PARAMS_PITCH_D         = 26,
        GYRO_PARAMS_PITCH_D_LIM     = 28,
        GYRO_PARAMS_PITCH_FILTER    = 30,

        GYRO_PARAMS_ROLL_P           = 32,
        GYRO_PARAMS_ROLL_P_LIM       = 34,
        GYRO_PARAMS_ROLL_I           = 36,
        GYRO_PARAMS_ROLL_I_LIM       = 38,
        GYRO_PARAMS_ROLL_D           = 40,
        GYRO_PARAMS_ROLL_D_LIM       = 42,
        GYRO_PARAMS_ROLL_FILTER      = 44,

        GYRO_PARAMS_YAW_P            = 46,
        GYRO_PARAMS_YAW_P_LIM        = 48,
        GYRO_PARAMS_YAW_I            = 50,
        GYRO_PARAMS_YAW_I_LIM        = 52,
        GYRO_PARAMS_YAW_D            = 54,
        GYRO_PARAMS_YAW_D_LIM        = 56,
        GYRO_PARAMS_YAW_FILTER       = 58,


        ACC_PARAMS_PITCH_P          = 60,
        ACC_PARAMS_PITCH_P_LIM      = 62,
        ACC_PARAMS_PITCH_I          = 64,
        ACC_PARAMS_PITCH_I_LIM      = 66,
        ACC_PARAMS_PITCH_D          = 68,
        ACC_PARAMS_PITCH_D_LIM      = 70,
        ACC_PARAMS_PITCH_FILTER     = 72,


        ACC_PARAMS_ROLL_P           = 74,
        ACC_PARAMS_ROLL_P_LIM       = 76,
        ACC_PARAMS_ROLL_I           = 78,
        ACC_PARAMS_ROLL_I_LIM       = 80,
        ACC_PARAMS_ROLL_D           = 82,
        ACC_PARAMS_ROLL_D_LIM       = 84,
        ACC_PARAMS_ROLL_FILTER      = 86,


        ACC_PARAMS_Z_P              = 88,
        ACC_PARAMS_Z_P_LIM          = 90,
        ACC_PARAMS_Z_I              = 92,
        ACC_PARAMS_Z_I_LIM          = 94,
        ACC_PARAMS_Z_D              = 96,
        ACC_PARAMS_Z_D_LIM          = 98,
        ACC_PARAMS_Z_FILTER         = 100,

        // Altitude Sonar
        SNR_PARAMS_Z_P              = 102,            
        SNR_PARAMS_Z_P_LIM          = 104,
        SNR_PARAMS_Z_I              = 106,
        SNR_PARAMS_Z_I_LIM          = 108,
        SNR_PARAMS_Z_D              = 110,
        SNR_PARAMS_Z_D_LIM          = 112,
        SNR_PARAMS_Z_FILTER         = 114,


        //ACC_PARAMS          = 42,
        VOLTAGE_ALARM               = 116,
        VOLTAGE_PITCH_TRIM          = 118,
        VOLTAGE_ROLL_TRIM           = 120,
    }


    public class HefnyCopterBaseConnection
    {


        public delegate void delegate_CopyData(SerialPacket oSerialPacket);


        #region "Attributes"


        public delegate_CopyData mdelegate_CopyData;

        protected string mReceivedData;

        #endregion 



        #region "Properties"
        #endregion 



        #region "Constructors"

        protected HefnyCopterBaseConnection()
        {
        }

        #endregion 


        #region "Methods"


        public virtual void Open()
        {
            throw new NotImplementedException();
        }


        public virtual void Close()
        {
            throw new NotImplementedException();
        }

        public virtual void SendCommand(ENUM_SerialCommands SerialCommand)
        {
            SendCommand(new HefnyCopterCommand(SerialCommand));
        }

        /// <summary>
        /// Send Command -either CMD or SETVALUE- to vehicle.
        /// </summary>
        /// <param name="HefnyCopterCommand"></param>
        public virtual void SendCommand(HefnyCopterCommand HefnyCopterCommand)
        {
            return;
        }

        protected virtual void CopyData(SerialPacket oSerialPacket)
        {
            throw new NotImplementedException();
        }

        
        #endregion

    }
}
