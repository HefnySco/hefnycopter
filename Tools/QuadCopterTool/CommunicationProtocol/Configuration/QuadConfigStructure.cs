using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol
{
    public  class QuadConfigStructure
    {

        #region "Attributes"

        protected static PIDParameters[] mGyroParams , mAccParams, mSonarParams ;

        #endregion 

        #region "Properties"

        public  byte Signature
        {
            set; get;
        }

        public  byte IsCalibrated
        {
            set;
            get;
        }

        public  byte RX_Mode
        {
            set;
            get;
        }

        public  byte SelfLevelingMode
        {
            set;
            get;
        }

        public  byte ArmingMode
        {
            set;
            get;
        }

        public  byte AutoDisarm
        {
            set;
            get;
        }


        public  byte IsECSCalibration
        {
            set;
            get;
        }

        public  byte ReceiverMode
        {
            set;
            get;
        }

        public  byte MixerIndex
        {
            set;
            get;
        }


        public  byte QuadFlyingMode
        {
            set;
            get;
        }


        public  byte LCDContrast
        {
            set;
            get;
        }

        public  byte HeightDamping
        {
            set;
            get;
        }

        public  byte HeightDampingLimit
        {
            set;
            get;
        }

        public  byte LVA
        {
            set;
            get;
        }

        public  PIDParameters[] GyroParams
        {
            get
            {
                return mGyroParams; 
            }
        }

        public  PIDParameters[] AccParams
        {
            get
            {
                return mAccParams;
            }
        }

        public  PIDParameters[] SonarParams
        {
            get
            {
                return mSonarParams;
            }
        }

        public  byte VoltageAlarm
        {
            set;
            get;
        }

        #endregion 


        #region "Constructors"

        public QuadConfigStructure()
        {
            mGyroParams = new PIDParameters[2];
            mAccParams = new PIDParameters[2];
            mSonarParams = new PIDParameters[1];

            mSonarParams[0] = new PIDParameters();
            for (int i = 0; i < 2; ++i)
            {
                mGyroParams[i] = new PIDParameters();
                mAccParams[i] = new PIDParameters();
                
            }
                
        }

        #endregion 
    }
}
