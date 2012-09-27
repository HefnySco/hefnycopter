namespace XBEECommunication
{
    partial class MainForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.label1 = new System.Windows.Forms.Label();
            this.cmbCOMPort = new System.Windows.Forms.ComboBox();
            this.btnPortOpen = new System.Windows.Forms.Button();
            this.btnPortClose = new System.Windows.Forms.Button();
            this.XBEEPort = new System.IO.Ports.SerialPort(this.components);
            this.txtReceivedData = new System.Windows.Forms.TextBox();
            this.txtSentData = new System.Windows.Forms.TextBox();
            this.btnSend = new System.Windows.Forms.Button();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.btnClear = new System.Windows.Forms.Button();
            this.txtGyroX = new System.Windows.Forms.TextBox();
            this.txtGyroY = new System.Windows.Forms.TextBox();
            this.txtGyroZ = new System.Windows.Forms.TextBox();
            this.txtAccX = new System.Windows.Forms.TextBox();
            this.txtAccY = new System.Windows.Forms.TextBox();
            this.txtAccZ = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(13, 13);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(57, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "COM Port:";
            // 
            // cmbCOMPort
            // 
            this.cmbCOMPort.FormattingEnabled = true;
            this.cmbCOMPort.Location = new System.Drawing.Point(77, 13);
            this.cmbCOMPort.Name = "cmbCOMPort";
            this.cmbCOMPort.Size = new System.Drawing.Size(121, 21);
            this.cmbCOMPort.TabIndex = 1;
            // 
            // btnPortOpen
            // 
            this.btnPortOpen.Location = new System.Drawing.Point(292, 11);
            this.btnPortOpen.Name = "btnPortOpen";
            this.btnPortOpen.Size = new System.Drawing.Size(75, 23);
            this.btnPortOpen.TabIndex = 2;
            this.btnPortOpen.Text = "Open Port";
            this.btnPortOpen.UseVisualStyleBackColor = true;
            this.btnPortOpen.Click += new System.EventHandler(this.btnPortOpen_Click);
            // 
            // btnPortClose
            // 
            this.btnPortClose.Location = new System.Drawing.Point(373, 12);
            this.btnPortClose.Name = "btnPortClose";
            this.btnPortClose.Size = new System.Drawing.Size(75, 23);
            this.btnPortClose.TabIndex = 3;
            this.btnPortClose.Text = "Close Port";
            this.btnPortClose.UseVisualStyleBackColor = true;
            this.btnPortClose.Click += new System.EventHandler(this.btnPortClose_Click);
            // 
            // XBEEPort
            // 
            this.XBEEPort.BaudRate = 57600;
            this.XBEEPort.DiscardNull = true;
            this.XBEEPort.RtsEnable = true;
            this.XBEEPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.XBEEPort_DataReceived);
            // 
            // txtReceivedData
            // 
            this.txtReceivedData.Location = new System.Drawing.Point(16, 212);
            this.txtReceivedData.Multiline = true;
            this.txtReceivedData.Name = "txtReceivedData";
            this.txtReceivedData.Size = new System.Drawing.Size(432, 165);
            this.txtReceivedData.TabIndex = 4;
            // 
            // txtSentData
            // 
            this.txtSentData.Location = new System.Drawing.Point(18, 78);
            this.txtSentData.Name = "txtSentData";
            this.txtSentData.Size = new System.Drawing.Size(349, 20);
            this.txtSentData.TabIndex = 5;
            // 
            // btnSend
            // 
            this.btnSend.Location = new System.Drawing.Point(373, 76);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(75, 23);
            this.btnSend.TabIndex = 6;
            this.btnSend.Text = "Send";
            this.btnSend.UseVisualStyleBackColor = true;
            this.btnSend.Click += new System.EventHandler(this.btnSend_Click);
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // btnClear
            // 
            this.btnClear.Location = new System.Drawing.Point(373, 126);
            this.btnClear.Name = "btnClear";
            this.btnClear.Size = new System.Drawing.Size(75, 23);
            this.btnClear.TabIndex = 7;
            this.btnClear.Text = "Clear";
            this.btnClear.UseVisualStyleBackColor = true;
            this.btnClear.Click += new System.EventHandler(this.btnClear_Click);
            // 
            // txtGyroX
            // 
            this.txtGyroX.Location = new System.Drawing.Point(77, 114);
            this.txtGyroX.Name = "txtGyroX";
            this.txtGyroX.Size = new System.Drawing.Size(100, 20);
            this.txtGyroX.TabIndex = 8;
            // 
            // txtGyroY
            // 
            this.txtGyroY.Location = new System.Drawing.Point(77, 140);
            this.txtGyroY.Name = "txtGyroY";
            this.txtGyroY.Size = new System.Drawing.Size(100, 20);
            this.txtGyroY.TabIndex = 8;
            // 
            // txtGyroZ
            // 
            this.txtGyroZ.Location = new System.Drawing.Point(77, 166);
            this.txtGyroZ.Name = "txtGyroZ";
            this.txtGyroZ.Size = new System.Drawing.Size(100, 20);
            this.txtGyroZ.TabIndex = 8;
            // 
            // txtAccX
            // 
            this.txtAccX.Location = new System.Drawing.Point(234, 114);
            this.txtAccX.Name = "txtAccX";
            this.txtAccX.Size = new System.Drawing.Size(100, 20);
            this.txtAccX.TabIndex = 8;
            // 
            // txtAccY
            // 
            this.txtAccY.Location = new System.Drawing.Point(234, 140);
            this.txtAccY.Name = "txtAccY";
            this.txtAccY.Size = new System.Drawing.Size(100, 20);
            this.txtAccY.TabIndex = 8;
            // 
            // txtAccZ
            // 
            this.txtAccZ.Location = new System.Drawing.Point(234, 166);
            this.txtAccZ.Name = "txtAccZ";
            this.txtAccZ.Size = new System.Drawing.Size(100, 20);
            this.txtAccZ.TabIndex = 8;
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(460, 393);
            this.Controls.Add(this.txtAccZ);
            this.Controls.Add(this.txtAccY);
            this.Controls.Add(this.txtAccX);
            this.Controls.Add(this.txtGyroZ);
            this.Controls.Add(this.txtGyroY);
            this.Controls.Add(this.txtGyroX);
            this.Controls.Add(this.btnClear);
            this.Controls.Add(this.btnSend);
            this.Controls.Add(this.txtSentData);
            this.Controls.Add(this.txtReceivedData);
            this.Controls.Add(this.btnPortClose);
            this.Controls.Add(this.btnPortOpen);
            this.Controls.Add(this.cmbCOMPort);
            this.Controls.Add(this.label1);
            this.Name = "MainForm";
            this.Text = "XBEE-Commnuicator";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.MainForm_FormClosing);
            this.Load += new System.EventHandler(this.MainForm_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox cmbCOMPort;
        private System.Windows.Forms.Button btnPortOpen;
        private System.Windows.Forms.Button btnPortClose;
        private System.IO.Ports.SerialPort XBEEPort;
        private System.Windows.Forms.TextBox txtReceivedData;
        private System.Windows.Forms.TextBox txtSentData;
        private System.Windows.Forms.Button btnSend;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Button btnClear;
        private System.Windows.Forms.TextBox txtGyroX;
        private System.Windows.Forms.TextBox txtGyroY;
        private System.Windows.Forms.TextBox txtGyroZ;
        private System.Windows.Forms.TextBox txtAccX;
        private System.Windows.Forms.TextBox txtAccY;
        private System.Windows.Forms.TextBox txtAccZ;
    }
}

