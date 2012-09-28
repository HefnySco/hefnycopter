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

namespace QuadCopterTool
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        System.Windows.Threading.DispatcherTimer mTimer;
        //public delegate void delegate_SensorData(Int16 GyroX, Int16 GyroY, Int16 GyroZ,Int16 AccX, Int16 AccY, Int16 AccZ);

        protected HefnyCopterSerial mHefnyCopterSerial;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            foreach (string portname in SerialPort.GetPortNames())
            {
                cmbCOMPorts.Items.Add(portname);
            }
            btnConnect.Tag = true;
            mHefnyCopterSerial = new HefnyCopterSerial();
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

        #endregion

        #region "tabData"

        private void Timer_Elapsed(object sender, EventArgs e)
        {
            //txtReceivedData.Text = SB.ToString();

            ctrlACC.RollLevel = mHefnyCopterSerial.AccY * 0.01256;
            ctrlACC.PitchLevel = mHefnyCopterSerial.AccX * 0.01256;
            //ctrlACC.YawLevel = (mHefnyCopterSerial.AccZ -100)* 0.01256;
            meterAccZ.CurrentValue = mHefnyCopterSerial.AccZ;


            ctrlGyro.RollLevel += mHefnyCopterSerial.GyroX  * 0.001256;
            ctrlGyro.PitchLevel += mHefnyCopterSerial.GyroY * 0.001256;
            //ctrlGyro.YawLevel += mHefnyCopterSerial.GyroZ  * 0.001256;
            meterGyroX.CurrentValue = mHefnyCopterSerial.GyroX;
            meterGyroY.CurrentValue = mHefnyCopterSerial.GyroY;

        }


        #endregion
    }
}
