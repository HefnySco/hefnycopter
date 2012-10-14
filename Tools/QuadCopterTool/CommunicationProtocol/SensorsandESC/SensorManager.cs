using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol.Sensors
{
    public class SensorManager
    {

      

        protected static GyroSensor[] mGyros;
        protected static AccelerometerSensor[] mAccs;
        protected static MotorESC[] mMotors;

        public static MotorESC[] Motors
        {
            get
            {
                return mMotors;
            }
        }

        public static GyroSensor Gyro_X
        {
            get
            {
                return mGyros[(int) ENUM_Gyro.Gyro_X];
            }
        }

        public static GyroSensor Gyro_Y
        {
            get
            {
                return mGyros[(int)ENUM_Gyro.Gyro_Y];
            }
        }

        public static GyroSensor Gyro_Z
        {
            get
            {
                return mGyros[(int)ENUM_Gyro.Gyro_Z];
            }
        }


        public static AccelerometerSensor Acc_X
        {
            get
            {
                return mAccs[(int)ENUM_Acc.Acc_X];
            }
        }

        public static AccelerometerSensor Acc_Y
        {
            get
            {
                return mAccs[(int)ENUM_Acc.Acc_Y];
            }
        }

        public static AccelerometerSensor Acc_Z
        {
            get
            {
                return mAccs[(int)ENUM_Acc.Acc_Z];
            }
        }





        public static void Init()
        {
            mGyros = new GyroSensor[3];
            mAccs = new AccelerometerSensor[3];
            mMotors = new MotorESC[4];


            for (int i = 0; i < 4; ++i)
            {
                mMotors[i] = new MotorESC();
            }


            for (int i = 0; i < 3; ++i)
            {
                mGyros[i] = new GyroSensor();
            }

            for (int i = 0; i < 3; ++i)
            {
                mAccs[i] = new AccelerometerSensor();
            }
        }
    }
}
