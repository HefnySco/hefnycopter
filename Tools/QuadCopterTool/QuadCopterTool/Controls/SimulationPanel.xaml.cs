using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Forms;

using QuadCopterTool.Misc;
using HefnyCopter.CommunicationProtocol;
using HefnyCopter.CommunicationProtocol.Sensors;


namespace QuadCopterTool
{
    /// <summary>
    /// Interaction logic for SimulationPanel.xaml
    /// </summary>
    public partial class SimulationPanel : System.Windows.Controls.UserControl
    {


        #region "Control Custom Events"
        public event EventHandler OnFileOpen;
        #endregion

        #region "Attributes"

        /// <summary>
        /// Timer Thread
        /// </summary>
        protected System.Windows.Threading.DispatcherTimer mTimer;
        protected StreamReader mStreamReader;
        protected bool mValidFileExist;

        protected bool mUpdateSldProgress;
        protected int mDelay;
        /// <summary>
        /// List of LogFileData from log file.
        /// </summary>
        protected List<LogFileData> mLogFileDataList;
        protected Int32 mCurrentValueIndex;
        #endregion

        #region "Properties"

        public Int32 SimulationDuration
        {
            get
            {
                if (mLogFileDataList.Count == 0)
                {
                    return -1;
                }
                else
                {
                    return mLogFileDataList[mLogFileDataList.Count - 1].Time - mLogFileDataList[0].Time;
                }


            }
        }

        #endregion

        public SimulationPanel()
        {
            InitializeComponent();
            UpdateButtonStatus();
            mLogFileDataList = new List<LogFileData>();
            mTimer = new System.Windows.Threading.DispatcherTimer();
            mTimer.Tick += new EventHandler(Timer_Elapsed);
        }


        protected void UpdateButtonStatus()
        {
            if (mValidFileExist == true)
            {
                btnRun.IsEnabled = true;
                btnBack.IsEnabled = true;
                btnForward.IsEnabled = true;
                btnPause.IsEnabled = true;
                btnStop.IsEnabled = true;
                sldProgress.IsEnabled = true;
                sldProgress.Minimum = 0;
                sldProgress.Maximum = mLogFileDataList.Count;
            }
            else
            {
                btnRun.IsEnabled = false;
                btnBack.IsEnabled = false;
                btnForward.IsEnabled = false;
                btnPause.IsEnabled = false;
                btnStop.IsEnabled = false;
                sldProgress.IsEnabled = false;
            }

        }

        protected void Timer_Elapsed(object sender, EventArgs e)
        {

            if (mCurrentValueIndex < mLogFileDataList.Count - 1)
            {
                mTimer.Interval = TimeSpan.FromMilliseconds(mDelay);// TimeSpan.FromMilliseconds(100 * (mLogFileDataList[mCurrentValueIndex + 1].Time - mLogFileDataList[mCurrentValueIndex].Time));
                lblDuration.Content = (mLogFileDataList[mCurrentValueIndex].Time-mLogFileDataList[0].Time).ToString() + ":" + (SimulationDuration).ToString() + " ms";
             
            }
            else
            {
                mTimer.Stop();
            }
            SensorManager.Gyro_X.AddValue(mLogFileDataList[mCurrentValueIndex].Gyro_X);
            SensorManager.Gyro_Y.AddValue(mLogFileDataList[mCurrentValueIndex].Gyro_Y);
            SensorManager.Gyro_Z.AddValue(mLogFileDataList[mCurrentValueIndex].Gyro_Z);
            SensorManager.Acc_X.AddValue(mLogFileDataList[mCurrentValueIndex].Acc_X);
            SensorManager.Acc_Y.AddValue(mLogFileDataList[mCurrentValueIndex].Acc_Y);
            SensorManager.Acc_Z.AddValue(mLogFileDataList[mCurrentValueIndex].Acc_Z);
            SensorManager.Motors[0].AddValue(mLogFileDataList[mCurrentValueIndex].Motor1);
            SensorManager.Motors[1].AddValue(mLogFileDataList[mCurrentValueIndex].Motor2);
            SensorManager.Motors[2].AddValue(mLogFileDataList[mCurrentValueIndex].Motor3);
            SensorManager.Motors[3].AddValue(mLogFileDataList[mCurrentValueIndex].Motor4);

            SensorManager.Gyro_X.AddGraphValue();
            SensorManager.Gyro_Y.AddGraphValue();
            SensorManager.Gyro_Z.AddGraphValue();
            SensorManager.Acc_X.AddGraphValue();
            SensorManager.Acc_Y.AddGraphValue();
            SensorManager.Acc_Z.AddGraphValue();

            mCurrentValueIndex += 1;

            if (mUpdateSldProgress == true)
            {
                sldProgress.Value = mCurrentValueIndex;
            }


        }

        private void btnRun_Click(object sender, RoutedEventArgs e)
        {
            mCurrentValueIndex = 0;
            mTimer.Start();
        }

        private void btnPause_Click(object sender, RoutedEventArgs e)
        {
            if (mTimer.IsEnabled)
            {
                mTimer.Stop();
            }
            else
            {
                mTimer.Start();
            }
        }

        private void btnBack_Click(object sender, RoutedEventArgs e)
        {
            if (mDelay == 0) return;

            mDelay -= 10;
        }

        private void btnForward_Click(object sender, RoutedEventArgs e)
        {
            mTimer.Stop();
            mCurrentValueIndex = 0;
            mDelay = 0;
        }

        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            mDelay += 10;
        }

        private void btnOpen_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Forms.OpenFileDialog oOpenFileDlg = new OpenFileDialog();
            oOpenFileDlg.DefaultExt = @"*.csv";
            oOpenFileDlg.Filter = @"Log (*.csv)|*.csv|All Files (*.*)|*.*";
            if (oOpenFileDlg.ShowDialog() == DialogResult.OK)
            {
                if (File.Exists(oOpenFileDlg.FileName) == false)
                {
                    System.Windows.Forms.MessageBox.Show("File not found", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                mLogFileDataList.Clear(); // delete current data anyway even if file is not valie
                mStreamReader = File.OpenText(oOpenFileDlg.FileName);

                String S = mStreamReader.ReadLine();
                String[] Fields = S.Split(',');
                if ((Fields.Length != 11) || (Fields[0] != "Time"))
                {
                    System.Windows.Forms.MessageBox.Show("Bad File Format.\r\nFile may be corrupted.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    mStreamReader.Close();
                }
                else
                {
                    mValidFileExist = true;
                }

                while (mStreamReader.EndOfStream == false)
                {
                    S = mStreamReader.ReadLine();
                    Fields = S.Split(',');
                    LogFileData oLogFileData = new LogFileData(Fields);
                    mLogFileDataList.Add(oLogFileData);
                }


                UpdateButtonStatus();
                lblDuration.Content = "00:"+ (SimulationDuration / 1000).ToString() + " ms";
                mDelay = 0;
            }
        }



        private void sldProgress_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            mCurrentValueIndex = (int)sldProgress.Value;

        }

    }
}
