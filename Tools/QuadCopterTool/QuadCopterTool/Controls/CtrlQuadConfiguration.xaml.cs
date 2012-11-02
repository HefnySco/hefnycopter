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
            ctrlSensorAccZ.Parameters = mQuadConfigStructure.AccParams[1];
            ctrlSensorGyroPitchRoll.Parameters = mQuadConfigStructure.GyroParams[0];
            ctrlSensorGyroYaw.Parameters = mQuadConfigStructure.GyroParams[1];

        }
    }
}
