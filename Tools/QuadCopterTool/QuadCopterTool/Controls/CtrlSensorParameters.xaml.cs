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

using HefnyCopter.CommunicationProtocol;

namespace QuadCopterTool.Controls
{
    /// <summary>
    /// Interaction logic for CtrlSensorParameters.xaml
    /// </summary>
    public partial class CtrlSensorParameters : UserControl
    {

        protected PIDParameters mParameters;

        [Browsable(true), Category("Misc"), Description("Current Value")]
        public string Caption
        {
            get
            {
                return gbContainer.Header.ToString();
            }
            set
            {
                gbContainer.Header = value;
            }
        }

        public PIDParameters Parameters
        {
            get
            {
                CopyData();
                return mParameters;
            }

            set
            {
                mParameters=value;
                UpdateUI();
            }
        }

        public CtrlSensorParameters()
        {
            InitializeComponent();
        }



        protected void CopyData()
        {
            try
            {
                mParameters.P = short.Parse(txtP.Text);
                mParameters.I=short.Parse(txtI.Text);
                mParameters.D=short.Parse(txtD.Text);
                mParameters.P_Limit=short.Parse(txtPLimit.Text);
                mParameters.I_Limit=short.Parse(txtILimit.Text);
                mParameters.D_Limit=short.Parse(txtDLimit.Text);
                mParameters.ComplementartuFilterAlpha=short.Parse(txtAlpha.Text);

            }
            catch
            {
                if (DesignerProperties.GetIsInDesignMode(new DependencyObject()) == true)
                {
                    return;
                }

                MessageBox.Show("Error","Invalid values in " + gbContainer.Header.ToString(), MessageBoxButton.OK);
            }
        }

        protected void UpdateUI()
        {
            txtP.Text = mParameters.P.ToString();
            txtI.Text = mParameters.I.ToString();
            txtD.Text = mParameters.D.ToString();
            txtPLimit.Text = mParameters.P_Limit.ToString();
            txtILimit.Text = mParameters.I_Limit.ToString();
            txtDLimit.Text = mParameters.D_Limit.ToString();
            txtAlpha.Text = mParameters.ComplementartuFilterAlpha.ToString();
        }
    }
}
