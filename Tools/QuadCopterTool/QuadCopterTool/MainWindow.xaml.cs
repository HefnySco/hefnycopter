/*
 * 
 * 
 * 
 *  Aknowledgment to:
 *  http://d3future.codeplex.com/SourceControl/changeset/view/26c396f40289
 *  http://www.mesta-automation.com/real-time-line-charts-with-wpf-and-dynamic-data-display/
 *  
 */


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
using System.IO.Ports;
using System.IO;
using System.Windows.Forms;
using System.ComponentModel;


using Microsoft.Research.DynamicDataDisplay;
using Microsoft.Research.DynamicDataDisplay.DataSources;

using HefnyCopter.CommunicationProtocol;
using HefnyCopter.CommunicationProtocol.Sensors;



namespace QuadCopterTool
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        
      

        System.Windows.Threading.DispatcherTimer mTimer;
        //public delegate void delegate_SensorData(Int16 GyroX, Int16 GyroY, Int16 GyroZ,Int16 AccX, Int16 AccY, Int16 AccZ);

        protected HefnyCopterSerial mHefnyCopterSerial;

        public MainWindow()
        {
            InitializeComponent();

            SensorManager.Init();
            // Create first source
            var source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_X);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotter.AddLineGraph(source, Colors.Magenta, 2, "Gyro Roll");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_Y);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotter.AddLineGraph(source, Colors.CadetBlue, 2, "Gyro Pitch");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_Z);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotter.AddLineGraph(source, Colors.MediumPurple, 2, "Gyro YAW");

            //chartPlotter.InputBindings


        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

            this.DataContext = this;


            foreach (string portname in SerialPort.GetPortNames())
            {
                cmbCOMPorts.Items.Add(portname);
            }


         
         
           
            btnConnect.Tag = true;
            mHefnyCopterSerial = new HefnyCopterSerial();
            mHefnyCopterSerial.mdelegate_CopyData = CopyData;
            mTimer = new System.Windows.Threading.DispatcherTimer();
            //mTimer.AutoReset = true;
            mTimer.Interval = TimeSpan.FromMilliseconds(100); 
            mTimer.Tick += new EventHandler(Timer_Elapsed);
            mTimer.Start();
        }

        #region "tabConnection"

        private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            if ((bool) btnConnect.Tag == true)
            {
                mHefnyCopterSerial.OpenPort(cmbCOMPorts.SelectedValue.ToString(), @"d:\QuadLog");
                btnConnect.Content = "Disconnect";
                btnConnect.Tag = false;
            }
            else
            {
                mHefnyCopterSerial.ClosePort();
                btnConnect.Content = "Connect";
                btnConnect.Tag = true;
            }
        }

        
        public void CopyData(byte[] vArray)
        {
            if (bRead == true)
            {
                bRead = false;
                chartPlotter.Dispatcher.BeginInvoke(new Action(delegate()
                    {
                        SensorManager.Gyro_X.AddGraphValue();
                        SensorManager.Gyro_Y.AddGraphValue();
                        SensorManager.Gyro_Z.AddGraphValue();
                        SensorManager.Acc_X.AddGraphValue();
                        SensorManager.Acc_Y.AddGraphValue();
                        SensorManager.Acc_Z.AddGraphValue();

                    }));
            }
        }
        #endregion

        #region "tabData"
        bool bRead = false;
        private void Timer_Elapsed(object sender, EventArgs e)
        {
            bRead = true;
         
            ctrlACC.RollLevel = -SensorManager.Acc_Y.LastValue * 0.01256; // mHefnyCopterSerial.AccY * 0.01256;
            ctrlACC.PitchLevel = -SensorManager.Acc_X.LastValue * 0.01256; // mHefnyCopterSerial.AccX * 0.01256;
            //ctrlACC.YawLevel = (mHefnyCopterSerial.AccZ -100)* 0.01256;
            meterAccZ.CurrentValue = (int) SensorManager.Acc_Z.LastValue;  // mHefnyCopterSerial.AccZ;


            meterGyroX.CurrentValue = (int)SensorManager.Gyro_X.LastValue; // mHefnyCopterSerial.GyroX;
            meterGyroY.CurrentValue = (int)SensorManager.Gyro_Y.LastValue; //mHefnyCopterSerial.GyroY;
            meterGyroZ.CurrentValue = (int)SensorManager.Gyro_Z.LastValue; //mHefnyCopterSerial.GyroZ;

            meterMotor1.CurrentValue = SensorManager.Motors[0].LastValue;
            meterMotor2.CurrentValue = SensorManager.Motors[1].LastValue;
            meterMotor3.CurrentValue = SensorManager.Motors[2].LastValue;
            meterMotor4.CurrentValue = SensorManager.Motors[3].LastValue;

             
         }


        #endregion

        #region INotifyPropertyChanged members

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
                this.PropertyChanged(this, new System.ComponentModel.PropertyChangedEventArgs(propertyName));
        }

        #endregion
    }
}
