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

using HefnyCopter.CommunicationProtocol;

namespace QuadCopterTool.Controls
{
    /// <summary>
    /// Interaction logic for CtrlQuadConfiguration.xaml
    /// </summary>
    public partial class CtrlQuadConfiguration : UserControl
    {
        public event EventHandler OnReadRequest;
        public event EventHandler OnWriteRequest;
        public event EventHandler OnSaveEPROMRequest;

       

        protected QuadConfigStructure mQuadConfigStructure;

        public QuadConfigStructure QuadConfigStructure
        {
            get
            {
                return mQuadConfigStructure;
            }

            set
            {
                mQuadConfigStructure = value;
            }
        }

        public CtrlQuadConfiguration()
        {
            InitializeComponent();
        }

        private void txtVoltageAlram_TextChanged(object sender, TextChangedEventArgs e)
        {
            short outResult;
            if (Int16.TryParse(txtVoltageAlram.Text, out  outResult) == false)
            {
                txtVoltageAlram.Tag = false;
                txtVoltageAlram.Foreground = new SolidColorBrush(Colors.Red);
            }
            else
            {
                txtVoltageAlram.Tag = true;
                txtVoltageAlram.Foreground = new SolidColorBrush(Colors.Black);
            }
        }

        private void Grid_Loaded(object sender, RoutedEventArgs e)
        {
            txtVoltageAlram.Tag = false;
        }

        public void UpdateUIValues()
        {
            txtVoltageAlram.Text = mQuadConfigStructure.VoltageAlarm.ToString();
            ctrlSensorAccPitchRoll.Parameters = mQuadConfigStructure.AccParams[0];
            ctrlSensorAccRoll.Parameters = mQuadConfigStructure.AccParams[1];
            ctrlSensorAccZ.Parameters = mQuadConfigStructure.AccParams[2];

            ctrlSensorGyroPitchRoll.Parameters = mQuadConfigStructure.GyroParams[0];
            ctrlSensorGyroRoll.Parameters = mQuadConfigStructure.GyroParams[1];
            ctrlSensorGyroYaw.Parameters = mQuadConfigStructure.GyroParams[2];

            ctrlSensorSonar.Parameters = mQuadConfigStructure.SonarParams[0];

        }


        public void UpdateQuadConfigStructure()
        {
            mQuadConfigStructure.VoltageAlarm  = byte.Parse(txtVoltageAlram.Text);
            mQuadConfigStructure.AccParams[0]  = ctrlSensorAccPitchRoll.Parameters;
            mQuadConfigStructure.AccParams[1] = ctrlSensorAccRoll.Parameters;
            mQuadConfigStructure.AccParams[2] = ctrlSensorAccZ.Parameters;
            
            mQuadConfigStructure.GyroParams[0] = ctrlSensorGyroPitchRoll.Parameters;
            mQuadConfigStructure.GyroParams[1] = ctrlSensorGyroRoll.Parameters;
            mQuadConfigStructure.GyroParams[2] = ctrlSensorGyroYaw.Parameters;

            mQuadConfigStructure.SonarParams[0] = ctrlSensorSonar.Parameters;
        }

        private void btnRead_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                btnRead.IsEnabled = false;
                //  (((sender as Button).Parent as Grid).Parent as CustomButton).Click(sender, e);
                OnReadRequest(sender, e);
            }
            catch
            {
            }
            finally
            {
                btnRead.IsEnabled = true;
            }
        }

        private void btnWrite_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                btnWrite.IsEnabled = false;
                UpdateQuadConfigStructure();
                OnWriteRequest(sender, e);
            }
            catch
            {
            }
            finally
            {
                btnWrite.IsEnabled = true;
            }
        }

        private void btnSaveEPROM_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                btnSaveEPROM.IsEnabled = false;
                OnSaveEPROMRequest(sender, e);
            }
            catch
            {
            }
            finally
            {
                btnSaveEPROM.IsEnabled = true;
            }
        }
    }
}
