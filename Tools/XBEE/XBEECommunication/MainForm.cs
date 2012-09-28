using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;

namespace XBEECommunication
{
    public partial class MainForm : Form
    {
        System.IO.TextWriter mFile;        
        Int16 GyroY, GyroX, GyroZ, AccY, AccX, AccZ;
        byte[] vArray;
        System.Text.StringBuilder SB = new StringBuilder();
        
        public MainForm()
        {
            InitializeComponent();
        }


        protected void ClosePort()
        {
            mFile.Close();
            if (XBEEPort.IsOpen == true)
            {
                XBEEPort.Close();
                //XBEEPort = null;
            }
        }

        protected void OpenPort()
        {
            if (XBEEPort.IsOpen == false)
            {
                mFile = System.IO.File.CreateText(@"d:\QuadReadings_" + System.Environment.TickCount.ToString()+".csv");
                XBEEPort.DiscardNull = false;
                XBEEPort.Encoding = Encoding.Unicode;
                XBEEPort.PortName = (string)cmbCOMPort.SelectedItem;
                XBEEPort.Open();
            }
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            vArray=new byte[14];
            foreach (string portname in SerialPort.GetPortNames())
            {
                cmbCOMPort.Items.Add(portname);
            }

            timer1.Start();
         
        }

        private void btnPortOpen_Click(object sender, EventArgs e)
        {
            OpenPort();
        }

        private void btnPortClose_Click(object sender, EventArgs e)
        {
            ClosePort();
     
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            ClosePort();
        }

        bool ExpectEOF=false;
        bool bStartCopy = false;
        int Idx = 0;
        private void XBEEPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            
            string S;
            //SB.Clear();
            S= (XBEEPort.ReadExisting());

            byte[] array =  new byte[8000]; //

            array = Encoding.Unicode.GetBytes(S);
                int j;
           /* for (j = 0; j < XBEEPort.BytesToRead; ++j)
            {

                array[j] =(byte) XBEEPort.ReadByte();
            }
           */
           // XBEEPort.Read(array,0,XBEEPort.BytesToWrite);
            for (int i = 0; i < array.Length; ++i) // j; ++i)
            {
                if (array[i] == 'S')
                {
                   bStartCopy = true;
                   Idx = 0;
                   //i += 1;
                    continue;
                }
                if (array[i] == 'E')
                {
                    if (ExpectEOF == true) // message correct then copy
                    {
                        GyroY = BitConverter.ToInt16(vArray, 0);
                        GyroZ = BitConverter.ToInt16(vArray, 2);
                        GyroX = BitConverter.ToInt16(vArray, 4);
                        AccX = BitConverter.ToInt16(vArray, 6);
                        AccY = BitConverter.ToInt16(vArray, 8);
                        AccZ = BitConverter.ToInt16(vArray, 10);
                        mFile.Write(GyroX);
                        mFile.Write(",");
                        mFile.Write(GyroY);
                        mFile.Write(",");
                        mFile.Write(GyroZ);
                        mFile.Write(",");
                        mFile.Write(AccX);
                        mFile.Write(",");
                        mFile.Write(AccY);
                        mFile.Write(",");
                        mFile.Write(AccZ);
                        mFile.WriteLine();
                    }
                    ExpectEOF = false;
                   continue;
                }
                if (bStartCopy)
                {
                    if (Idx == 14)
                    {
                        Idx = 0;
                        bStartCopy = false;
                        ExpectEOF = true;
                        i += 1;
                        continue;
                    }
                    vArray[Idx]=array[i];
                    Idx += 1;
                }

                ExpectEOF = false;
            }
            
          
        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            if (XBEEPort.IsOpen == true)
            {
                XBEEPort.Write(txtSentData.Text);

            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //txtReceivedData.Text = SB.ToString();

            txtAccX.Text = AccX.ToString();
            txtAccY.Text = AccY.ToString();
            txtAccZ.Text = AccZ.ToString();
            txtGyroX.Text = GyroX.ToString();
            txtGyroY.Text = GyroY.ToString();
            txtGyroZ.Text = GyroZ.ToString();
        }

        private void btnClear_Click(object sender, EventArgs e)
        {
            SB.Clear();
        }
    }
}
