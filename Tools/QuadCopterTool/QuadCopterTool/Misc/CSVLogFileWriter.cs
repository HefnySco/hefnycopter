using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


using HefnyCopter.CommunicationProtocol.Sensors;

namespace QuadCopterTool.Misc
{
    public class CSVLogFileWriter
    {

        #region "Attributes"

        protected System.IO.TextWriter mLogFile;
        protected string mFileName;

        #endregion 



        #region "Properties"

        
        public string FileName
        {
            get
            {
                return mFileName;
            }
        }
        #endregion 



        #region "Constructors"

        public CSVLogFileWriter(string FileName)
        {
            mFileName = FileName;
        }
        #endregion 


        #region "Methods"

        public void Create()
        {
            mLogFile = System.IO.File.CreateText(FileName);
            LogHeader();
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


        public void LogData()
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



        public void Close()
        {
            mLogFile.Close();
        }

        #endregion 

    }
}
