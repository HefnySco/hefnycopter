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
    /// Interaction logic for Meter.xaml
    /// </summary>
    public partial class Meter : Canvas
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
                if (rectangle1== null) return;
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


        public Meter()
        {
            InitializeComponent();
        }

        private void Grid_Loaded(object sender, RoutedEventArgs e)
        {

            //Canvas.SetBottom(rectangle2, this.Height);
            //Canvas.SetLeft(rectangle2, 0);
            //Canvas.SetTop(rectangle2, 0);
            //Canvas.SetRight(rectangle2, 0);

            mRatio = (this.Height / (mMaxValue - mMinValue));

            Draw();

        }

        protected void Draw()
        {
            rectangleValue.Height = mRatio * (mMaxValue - mCurrentValue);
            lblValue.Content = mCurrentValue ;
        }

        private void Canvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            mRatio = (this.Height / (mMaxValue - mMinValue));

            //Canvas.SetRight(MainGrid, this.Width);
            Draw();

          


            MainGrid.Width = this.Width;
            MainGrid.Height= this.Height;
            
            
     
        }

    }
}
