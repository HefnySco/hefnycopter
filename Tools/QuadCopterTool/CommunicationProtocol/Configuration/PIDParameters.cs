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


        public Int16 ComplementartuFilterAlpha
        {
            get;
            set;
        }

        #endregion 


      
    }
}
