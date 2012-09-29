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
    /// Interaction logic for DataPlotter.xaml
    /// </summary>
    public partial class DataPlotter : Canvas
    {
        Line mMiddleLine, mY_AxisLine,mX_AxisLine;
        Int16[] Data;
        public DataPlotter()
        {
            if (mX_AxisLine != null) return;

            Data = new Int16[1024];

            SolidColorBrush myBrush = new  SolidColorBrush(Colors.Green);
            
            mX_AxisLine = new Line();
            mX_AxisLine.Stroke = myBrush;
            mY_AxisLine = new Line();
            mY_AxisLine.Stroke = myBrush;
            mMiddleLine = new Line();
            mMiddleLine.Stroke = myBrush;

            this.Children.Add(mX_AxisLine);
            this.Children.Add(mY_AxisLine);
            this.Children.Add(mMiddleLine);

            InitializeComponent();
        }

        private void Canvas_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void Canvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            
            mMiddleLine.X1 = 0;
            mMiddleLine.Y1 = this.Height / 2;
            mMiddleLine.X2 = this.Width;
            mMiddleLine.Y2 = this.Height / 2; 

            mX_AxisLine.X1 = 0;
            mX_AxisLine.Y1 = this.Height - 5;
            mX_AxisLine.X2 = this.Width - 2;
            mX_AxisLine.Y2 = this.Height - 5;

            mY_AxisLine.X1 = 5;
            mY_AxisLine.Y1 = 0;
            mY_AxisLine.X2 = 5;
            mY_AxisLine.Y2 = this.Height;
        }
    }
}
