using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol
{

    public enum ENUM_RxDataType
    {
        Undefined,
        Sensors,
        Settings
    }

    public enum ENUM_SerialCommands
    {
        LED_BLINK     =0x1, // blink LED usually to know that connection is active
        READ_CONFIG   =0x5, // order vehicle to send config data.
        SAVE_CONFIG   =0x6, // save to EPROM
        CALIBRATE_ACC =0x7,
    }

    /// <summary>
    /// Value offset for each parameter in GlobalValues.h in HefnyCopter2
    /// </summary>
    public enum ENUM_Parameter 
    {
        AUTO_DISARM         = 5,
        BOARD_ORIENTATION   = 8,
        FLYING_MODE         = 9,
        THROTTLE_MIN        = 11, 
        STICK_SCALING       = 12,
        
        GYRO_PARAMS_PITCH_P         = 14,
        GYRO_PARAMS_PITCH_P_LIM     = 16,
        GYRO_PARAMS_PITCH_I         = 18,
        GYRO_PARAMS_PITCH_I_LIM     = 20,
        GYRO_PARAMS_PITCH_D         = 22,
        GYRO_PARAMS_PITCH_D_LIM     = 24,
        GYRO_PARAMS_PITCH_FILTER    = 26,

        GYRO_PARAMS_YAW_P           = 28,
        GYRO_PARAMS_YAW_P_LIM       = 30,
        GYRO_PARAMS_YAW_I           = 32,
        GYRO_PARAMS_YAW_I_LIM       = 34,
        GYRO_PARAMS_YAW_D           = 36,
        GYRO_PARAMS_YAW_D_LIM       = 38,
        GYRO_PARAMS_YAW_FILTER      = 40,


        ACC_PARAMS_PITCH_P          = 42,
        ACC_PARAMS_PITCH_P_LIM      = 44,
        ACC_PARAMS_PITCH_I          = 46,
        ACC_PARAMS_PITCH_I_LIM      = 48,
        ACC_PARAMS_PITCH_D          = 50,
        ACC_PARAMS_PITCH_D_LIM      = 52,
        ACC_PARAMS_PITCH_FILTER     = 54,

        ACC_PARAMS_Z_P              = 56,
        ACC_PARAMS_Z_P_LIM          = 58,
        ACC_PARAMS_Z_I              = 60,
        ACC_PARAMS_Z_I_LIM          = 62,
        ACC_PARAMS_Z_D              = 64,
        ACC_PARAMS_Z_D_LIM          = 66,
        ACC_PARAMS_Z_FILTER         = 68,

        // Altitude Sonar
        SNR_PARAMS_Z_P              = 70,            
        SNR_PARAMS_Z_P_LIM          = 72,
        SNR_PARAMS_Z_I              = 74,
        SNR_PARAMS_Z_I_LIM          = 76,
        SNR_PARAMS_Z_D              = 78,
        SNR_PARAMS_Z_D_LIM          = 80,
        SNR_PARAMS_Z_FILTER         = 82,


        ACC_PARAMS          = 42,
        VOLTAGE_ALARM       = 84,
    }


    public class HefnyCopterBaseConnection
    {
       

        public delegate void delegate_CopyData(ENUM_RxDataType DataType , byte[] DataArray);


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

        protected virtual void CopyData(ENUM_RxDataType DataType, byte[] vArray)
        {
            throw new NotImplementedException();
        }

        
        #endregion

    }
}
