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
using System.Net;
using System.Threading;

using Microsoft.Research.DynamicDataDisplay;
using Microsoft.Research.DynamicDataDisplay.DataSources;

using QuadCopterTool.Misc;
using QuadCopterTool.Misc.Video;
using HefnyCopter.CommunicationProtocol;
using HefnyCopter.CommunicationProtocol.Sensors;


namespace QuadCopterTool
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {


        protected byte[] mDummyData = new byte[20];

        System.Windows.Threading.DispatcherTimer mTimer;
        //public delegate void delegate_SensorData(Int16 GyroX, Int16 GyroY, Int16 GyroZ,Int16 AccX, Int16 AccY, Int16 AccZ);

        protected HefnyCopterBaseConnection mHefnyCopterSerial;
        protected VideoComponent mVideoComponent;
        protected VideoFileWriter mVideoFileWriter;
        protected CSVLogFileWriter mCSVLogFileWriter;

        public MainWindow()
        {
            InitializeComponent();



            SensorManager.Init();
            // Create first source
            var source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_X);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterGyro.AddLineGraph(source, Colors.Magenta, 2, "Gyro Roll");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_Y);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterGyro.AddLineGraph(source, Colors.CadetBlue, 2, "Gyro Pitch");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Gyro_Z);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterGyro.AddLineGraph(source, Colors.MediumPurple, 2, "Gyro YAW");



            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Acc_Y);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterAcc.AddLineGraph(source, Colors.Magenta, 2, "Acc Roll");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Acc_X);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterAcc.AddLineGraph(source, Colors.CadetBlue, 2, "ACC Pitch");

            source = new EnumerableDataSource<GraphValueUnit>(SensorManager.Acc_Z);
            source.SetXMapping(x => x.X);
            source.SetYMapping(y => y.Y);
            chartPlotterAcc.AddLineGraph(source, Colors.MediumPurple, 2, "Acc Z");


            txtVideoConnection.Text = Properties.Settings.Default["VideoConnection"].ToString();
            txtLogFolder.Text = Properties.Settings.Default["LogFolder"].ToString(); //Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);


        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

            this.DataContext = this;


            foreach (string portname in SerialPort.GetPortNames())
            {
                cmbCOMPorts.Items.Add(portname);
            }
            
            mVideoComponent = new VideoComponent(ImageReceived);
            btnConnect.Tag = true;
            mTimer = new System.Windows.Threading.DispatcherTimer();
            //mTimer.AutoReset = true;
            mTimer.Interval = TimeSpan.FromMilliseconds(100);
            mTimer.Tick += new EventHandler(Timer_Elapsed);
            mTimer.Start();
        }

        #region "tabConnection"

        private void btnLogBrowse_Click(object sender, RoutedEventArgs e)
        {
            FolderBrowserDialog oFolderBrowserDialog = new FolderBrowserDialog();
            if (oFolderBrowserDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                txtLogFolder.Text = oFolderBrowserDialog.SelectedPath;
            }
        }

        private void txtLogFolder_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.IO.Directory.Exists(txtLogFolder.Text) == false)
            {
                txtLogFolder.Foreground = Brushes.Red;
            }
            else
            {
                txtLogFolder.Foreground = Brushes.Black;
                Properties.Settings.Default["LogFolder"] = txtLogFolder.Text;
                Properties.Settings.Default.Save();
            }
        }

        private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if ((bool)btnConnect.Tag == true)
                {
                    string FileName, CSVFile, VideoFile;
                    FileName = txtLogFolder.Text + @"\" + Properties.Settings.Default["LogFileName"] + System.Environment.TickCount.ToString();
                    CSVFile = FileName + ".csv";
                    VideoFile = FileName + ".framed_vid";
                    
                    if (chkPortFake.IsChecked == true)
                    {
                        HefnyCopterDummySerial oHefnyCopterDummySerial = new HefnyCopterDummySerial();
                        mHefnyCopterSerial = oHefnyCopterDummySerial;
                        mHefnyCopterSerial.mdelegate_CopyData = CopyData;
                        
                    }
                    else
                    {
                        HefnyCopterSerial oHefnyCopterSerial = new HefnyCopterSerial();
                        mHefnyCopterSerial = oHefnyCopterSerial;
                        mHefnyCopterSerial.mdelegate_CopyData = CopyData;
                        oHefnyCopterSerial.PortName = cmbCOMPorts.SelectionBoxItem.ToString();
                        oHefnyCopterSerial.BaudRate = int.Parse(cmdBaudRate.SelectionBoxItem.ToString());
                        oHefnyCopterSerial.FileName = CSVFile;
                    }
                    mHefnyCopterSerial.Open();
                    chkPortFake.IsEnabled = false;


                    mCSVLogFileWriter = new CSVLogFileWriter(CSVFile);
                    mCSVLogFileWriter.Create();


                    mVideoFileWriter = new VideoFileWriter(VideoFile);
                    mVideoFileWriter.Create();
                    btnConnect.Content = "Disconnect";
                    btnConnect.Tag = false;
                    simulationPanel1.IsEnabled = false;
                }
                else
                {
                    
                    if (chkPortFake.IsChecked == true)
                    {
                        mHefnyCopterSerial.Close();
                    }
                    else
                    {
                        mHefnyCopterSerial.Close();
                    }

                    chkPortFake.IsEnabled = true;
                    mVideoFileWriter.Close();
                    mVideoFileWriter = null;
                    mCSVLogFileWriter.Close();
                    mCSVLogFileWriter=null;
                    btnConnect.Content = "Connect";
                    btnConnect.Tag = true;
                    simulationPanel1.IsEnabled = true;
                }
            }
            catch
            {
                System.Windows.Forms.MessageBox.Show("Serial Port Failed", "Connection Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }


        public void CopyData(byte[] vArray)
        {

            SensorManager.Gyro_X.AddValue(BitConverter.ToInt16(vArray, 0));
            SensorManager.Gyro_Y.AddValue(BitConverter.ToInt16(vArray, 2));
            SensorManager.Gyro_Z.AddValue(BitConverter.ToInt16(vArray, 4));
            SensorManager.Acc_X.AddValue(BitConverter.ToInt16(vArray, 6));
            SensorManager.Acc_Y.AddValue(BitConverter.ToInt16(vArray, 8));
            Int16 Z = (short) ((BitConverter.ToInt16(vArray, 10)) - 100);
            SensorManager.Acc_Z.AddValue(Z);

            SensorManager.Motors[0].AddValue(BitConverter.ToInt16(vArray, 12));
            SensorManager.Motors[1].AddValue(BitConverter.ToInt16(vArray, 14));
            SensorManager.Motors[2].AddValue(BitConverter.ToInt16(vArray, 16));
            SensorManager.Motors[3].AddValue(BitConverter.ToInt16(vArray, 18));

            // Log Data into CSV file
            mCSVLogFileWriter.LogData(); 


            // UPdate Graph based on Timer Rate not actual received data rate.
            if (bRead == true)
            {
                bRead = false;
                chartPlotterGyro.Dispatcher.BeginInvoke(new Action(delegate()
                    {
                        SensorManager.Gyro_X.AddGraphValue();
                        SensorManager.Gyro_Y.AddGraphValue();
                        SensorManager.Gyro_Z.AddGraphValue();
                    }));
                chartPlotterAcc.Dispatcher.BeginInvoke(new Action(delegate()
                {
                    SensorManager.Acc_X.AddGraphValue();
                    SensorManager.Acc_Y.AddGraphValue();
                    SensorManager.Acc_Z.AddGraphValue();

                }));
            }
        }


        #endregion

        #region "tabData"


        bool bRead = false;
        /// <summary>
        /// Called by Timer to update UI. UI update rate is less that actual data. You can review all data in simulation mode in much slower rate.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Timer_Elapsed(object sender, EventArgs e)
        {

            bRead = true;

            ctrlACC.RollLevel = -SensorManager.Acc_Y.LastValue * 0.01256; // mHefnyCopterSerial.AccY * 0.01256;
            ctrlACC.PitchLevel = -SensorManager.Acc_X.LastValue * 0.01256; // mHefnyCopterSerial.AccX * 0.01256;
            //ctrlACC.YawLevel = (mHefnyCopterSerial.AccZ -100)* 0.01256;
            meterAccZ.CurrentValue = (int)SensorManager.Acc_Z.LastValue;  // mHefnyCopterSerial.AccZ;


            meterGyroX.CurrentValue = (int)SensorManager.Gyro_X.LastValue; // mHefnyCopterSerial.GyroX;
            meterGyroY.CurrentValue = (int)SensorManager.Gyro_Y.LastValue; //mHefnyCopterSerial.GyroY;
            meterGyroZ.CurrentValue = (int)SensorManager.Gyro_Z.LastValue; //mHefnyCopterSerial.GyroZ;

            meterMotor1.CurrentValue = SensorManager.Motors[0].LastValue;
            meterMotor2.CurrentValue = SensorManager.Motors[1].LastValue;
            meterMotor3.CurrentValue = SensorManager.Motors[2].LastValue;
            meterMotor4.CurrentValue = SensorManager.Motors[3].LastValue;

        }


        #endregion


        #region "tabVideo"


        private void simulationPanel1_OnFileRun(object sender, EventArgs e)
        {
            mVideoComponent.Close();
        }

        /// <summary>
        /// called by simulator control
        /// </summary>
        /// <param name="memStream"></param>
        private void simulationPanel1_OnImageReceived(MemoryStream memStream)
        {
            ImageReceived(memStream);
        }

        /// <summary>
        /// called by Video Component to update a frame.
        /// </summary>
        /// <param name="memStream"></param>
        private void ImageReceived(MemoryStream memStream)
        {
            if (mVideoFileWriter != null)
            {
                mVideoFileWriter.AddFrame(memStream);
            }

            // since this is another thread you need to Dispatched to access UI.
            Dispatcher.Invoke((ThreadStart)delegate()
                {
                    memStream.Seek(0, SeekOrigin.Begin);
                    BitmapImage bmpCamera = new BitmapImage();
                    bmpCamera.BeginInit();
                    bmpCamera.StreamSource = memStream;
                    bmpCamera.CacheOption = BitmapCacheOption.OnLoad;
                    bmpCamera.EndInit();

                    imgLive.Source = bmpCamera;
                });
        }


        private void txtVideoConnection_TextChanged(object sender, TextChangedEventArgs e)
        {

            if (System.Uri.IsWellFormedUriString(txtVideoConnection.Text, UriKind.RelativeOrAbsolute) == false)
            {
                txtVideoConnection.Foreground = Brushes.Red;
            }
            else
            {
                txtLogFolder.Foreground = Brushes.Black;
                Properties.Settings.Default["VideoConnection"] = txtVideoConnection.Text;
                Properties.Settings.Default.Save();
            }

        }

        private void btnVideoOnOff_Click(object sender, RoutedEventArgs e)
        {
            ToggleVideoOnOff();
        }


        private void ToggleVideoOnOff()
        {
            if (mVideoComponent.Running == false)
            {
                mVideoComponent.Capture();
            }
            else
            {
                mVideoComponent.Close();

            }
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

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            mVideoComponent.Close();
        }

       

       





    }
}
