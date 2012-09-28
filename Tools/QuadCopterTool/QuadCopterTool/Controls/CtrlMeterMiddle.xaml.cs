using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;


namespace QuadCopterTool.Controls
{
    /// <summary>
    /// Interaction logic for CtrlMeterMiddle.xaml
    /// </summary>
    public partial class CtrlMeterMiddle : Canvas
    {

        protected int mMaxValue, mMinValue, mCurrentValue;

        protected double mRatio;



        [Browsable(true), Category("Misc"), Description("Current Value")]
        public int CurrentValue
        {
            get
            {
                return mCurrentValue;
            }
            set
            {
                if (value < mMinValue) return;
                if (value > mMaxValue) return;
                mCurrentValue = value;
                //if (rectangleValue == null) return;
                Draw();

            }
        }


        [Browsable(true), Category("Misc"), Description("Max Value")]
        public int MaxValue
        {
            get
            {
                return mMaxValue;
            }
            set
            {
                if (value < mMinValue) return;
                mMaxValue = value;
                mRatio = (this.Height / (mMaxValue - mMinValue));
            }
        }


        [Browsable(true), Category("Misc"), Description("Max Value")]
        public int MinValue
        {
            get
            {
                return mMinValue;
            }
            set
            {
                if (value > mMaxValue) return;
                mMinValue = value;
                mRatio = (this.Height / (mMaxValue - mMinValue));
            }
        }



        public CtrlMeterMiddle()
        {
            InitializeComponent();
        }

        private void Canvas_Loaded(object sender, RoutedEventArgs e)
        {
            mRatio = (this.Height / (mMaxValue - mMinValue));

            Draw();
        }

        protected void Draw()
        {
            int Middle = (mMaxValue + mMinValue) / 2;
            if (mCurrentValue >= Middle)
            {
                rectangleValueUp.Height = (mRatio *(mMaxValue-CurrentValue));
                //Canvas.SetTop(rectangleValueDown, Middle);
                rectangleValueDown.Height = mRatio * (Middle - mMinValue);
                //Canvas.SetBottom(rectangleValueUp, this.Height);
            }
            else
            {
                //Canvas.SetTop(rectangleValueUp, 0);
                rectangleValueUp.Height = mRatio * (mMaxValue - Middle);
                rectangleValueDown.Height = mRatio * (mCurrentValue-mMinValue);
                //Canvas.SetTop(rectangleValueDown, (mRatio * (mMaxValue - mCurrentValue)));
                //Canvas.SetBottom(rectangleValueDown, this.Height);
            }
            
            lblValue.Content = mCurrentValue;
        }

        private void Canvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            mRatio = (this.Height / (mMaxValue - mMinValue));

            MainGrid.Width = this.Width;
            MainGrid.Height = this.Height;
            
            Draw();




          



        }
    }
}
