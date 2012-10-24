using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace QuadCopterTool.Misc
{
    public class LogFileData
    {
        public Int32 Time;
        public Int16 Gyro_X, Gyro_Y, Gyro_Z, Acc_X, Acc_Y, Acc_Z, Motor1, Motor2, Motor3, Motor4;

        public LogFileData(string[] Data)
        {
            Time = int.Parse(Data[0]);
            Gyro_X = Int16.Parse(Data[1]);
            Gyro_Y = Int16.Parse(Data[2]);
            Gyro_Z = Int16.Parse(Data[3]);
            Acc_X = Int16.Parse(Data[4]);
            Acc_Y = Int16.Parse(Data[5]);
            Acc_Z = Int16.Parse(Data[6]);
            Motor1 = Int16.Parse(Data[7]);
            Motor2 = Int16.Parse(Data[8]);
            Motor3 = Int16.Parse(Data[9]);
            Motor4 = Int16.Parse(Data[10]);
            
        }
    }



}
