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

namespace QuadCopterTool
{
    /// <summary>
    /// Interaction logic for CtrlPosition.xaml
    /// </summary>
    public partial class CtrlPosition : Canvas
    {


        protected double const_Circle1_X = 45;
        protected double const_Circle1_Y = 72;
        protected double const_Circle1_Radius = 35;

        protected Line mRoll;
        protected Line mPitch;
        protected Line mYaw;

        protected double mRollLevel ;
        protected double mPitchLevel;
        protected double mYawLevel ;

        public double RollLevel
        {
            get { 
                return mRollLevel; 
                }
            set
            {
                try
                {
                    mRollLevel = value;

                    double invdValue = mRollLevel + Math.PI;
                    mRoll.X1 = ((Math.Cos(mRollLevel) * const_Circle1_Radius)) + const_Circle1_X;
                    mRoll.X2 = ((Math.Cos(invdValue) * const_Circle1_Radius)) + const_Circle1_X;
                    mRoll.Y1 = ((Math.Sin(mRollLevel) * const_Circle1_Radius)) + const_Circle1_Y;
                    mRoll.Y2 = ((Math.Sin(invdValue) * const_Circle1_Radius)) + const_Circle1_Y;
                }
                catch
                {
                    
                }
            }
        }

        public double PitchLevel
        {
            get { return mPitchLevel; }
            set
            {
                mPitchLevel = value;

                double invdValue = value + Math.PI;
                mPitch.X1 = ((Math.Cos(mPitchLevel) * const_Circle1_Radius)) + const_Circle1_X;
                mPitch.X2 = ((Math.Cos(invdValue) * const_Circle1_Radius)) + const_Circle1_X;
                mPitch.Y1 = ((Math.Sin(mPitchLevel) * const_Circle1_Radius)) + const_Circle1_Y;
                mPitch.Y2 = ((Math.Sin(invdValue) * const_Circle1_Radius)) + const_Circle1_Y;
         
            }
        }

        public double YawLevel
        {
            get { return mYawLevel - Math.PI/2; }
            set
            {
                mYawLevel = value + Math.PI/2;

                double invdValue = mYawLevel + Math.PI;
                mYaw.X1 = ((Math.Cos(mYawLevel) * const_Circle1_Radius)) + const_Circle1_X;
                mYaw.X2 = ((Math.Cos(invdValue) * const_Circle1_Radius)) + const_Circle1_X;
                mYaw.Y1 = ((Math.Sin(mYawLevel) * const_Circle1_Radius)) + const_Circle1_Y;
                mYaw.Y2 = ((Math.Sin(invdValue) * const_Circle1_Radius)) + const_Circle1_Y;
    
            }
        }

        [Browsable(true),Category("Misc"),Description("Control Title")]
        public string Title
        {
            get
            {
                return txtTitle.Text;
            }
            set
            {
                txtTitle.Text = value;
            }
        }


        public CtrlPosition()
        {
            InitializeComponent();
        }



        private void onMouseClick(object sender, MouseButtonEventArgs e)
        {

        }

        private void onLoaded(object sender, RoutedEventArgs e)
        {

            
            const_Circle1_Y = ellipeMain.ActualHeight / 2 + 27;
            const_Circle1_X = ellipeMain.ActualWidth / 2 + 10;
            const_Circle1_Radius = ellipeMain.ActualWidth/2-2;

            if (mRoll == null)
            {

                mRoll = new Line();
                mRoll.Stroke = Brushes.Red;
                mRoll.X1 = 3;
                mRoll.X2 = 53;
                mRoll.Y1 = 27;
                mRoll.Y2 = 27;
                mRoll.StrokeThickness = 1;

                this.Children.Add(mRoll);

                mPitch = new Line();
                mPitch.Stroke = Brushes.Blue;
                mPitch.X1 = 27;
                mPitch.X2 = 27;
                mPitch.Y1 = 3;
                mPitch.Y2 = 53;
                mPitch.StrokeThickness = 1;
                this.Children.Add(mPitch);


                mYaw = new Line();
                mYaw.Stroke = Brushes.Green;
                mYaw.X1 = 27;
                mYaw.X2 = 27;
                mYaw.Y1 = 3;
                mYaw.Y2 = 53;
                mYaw.StrokeThickness = 1;
                this.Children.Add(mYaw);
            }

            RollLevel = 0.0;
            YawLevel = 0.0;
            PitchLevel = 0.0;


            if (DesignerProperties.GetIsInDesignMode(this))
            {
                // we are in design mode.

            }
        }


        protected void DrawLine(Line oLine, double dValue)
        {
           
        }

    }
}
