using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.DynamicDataDisplay.Common;



namespace HefnyCopter.CommunicationProtocol.Sensors
{
    public class ValueBase : RingArray<GraphValueUnit>
    {

        #region "Attributes"

        protected const int TOTAL_POINTS = 100;
        private Int32 mInsertCount, mInsertCountGraph;
        protected Int16 mUpperLimit, mLowerLimit;
        protected Int16[] mValues;
        #endregion 


        #region "Properties"

        public Int16 UpperLimit
        {
            set
            {
                mUpperLimit = value;
            }
            get
            {
                return mUpperLimit;
            }
        }

        public Int16 LowerLimit
        {
            set
            {
                mLowerLimit = value;
            }
            get
            {
                return mLowerLimit;
            }
        }

        #endregion 

        public ValueBase(): base(TOTAL_POINTS) // here i set how much values to show 
        {
            mUpperLimit = 500;
            mLowerLimit = -500;
            mValues = new Int16[1000000];
        }


        /// <summary>
        /// Latest value 
        /// </summary>
        /// <param name="Y"></param>
        public void AddValue(Int16 Y)
        {
            if ((Y > mUpperLimit) || (Y < mLowerLimit))
            {
                Y = mValues[mInsertCount];
            }
            
            mInsertCount += 1;
            mValues[mInsertCount] = Y;
            //this.Add(new SensorValue(mInsertCount, Y));
            
        }

        public void AddGraphValue()
        {

            this.Add(new GraphValueUnit(mInsertCountGraph, mValues[mInsertCount]));
            mInsertCountGraph += 1;
        }

        public Int16 LastValue
        {
            get
            {
                return mValues[mInsertCount];
            }
        }
    }

  
}
