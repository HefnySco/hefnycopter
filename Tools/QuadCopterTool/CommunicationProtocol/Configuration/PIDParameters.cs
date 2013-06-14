using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HefnyCopter.CommunicationProtocol
{
    /// <summary>
    /// _P,_PLimit, 
	///	_I, _ILimit, 
	///	_D, _DLimit, 
	///	ComplementaryFilterAlpha
    /// </summary>
    public class PIDParameters
    {

        #region "Properties"

        public Int16 P
        {
            get;
            set;
        }

        public Int16 P_Limit
        {
            get;
            set;
        }

        public Int16 I
        {
            get;
            set;
        }

        public Int16 I_Limit
        {
            get;
            set;
        }


        public Int16 D
        {
            get;
            set;
        }


        public Int16 D_Limit
        {
            get;
            set;
        }


        public Int16 ComplementartyFilterAlpha
        {
            get;
            set;
        }

        #endregion 

        public void SetParameters(byte[] ParamValues,int Offset)
        {
            P = BitConverter.ToInt16(ParamValues, 0 + Offset);
            P_Limit = BitConverter.ToInt16(ParamValues, 2 + Offset);
            I = BitConverter.ToInt16(ParamValues, 4 + Offset);
            I_Limit = BitConverter.ToInt16(ParamValues, 6 + Offset);
            D = BitConverter.ToInt16(ParamValues, 8 + Offset);
            D_Limit = BitConverter.ToInt16(ParamValues, 10 + Offset);
            ComplementartyFilterAlpha = BitConverter.ToInt16(ParamValues, 12 + Offset);
        }

      
    }
}
