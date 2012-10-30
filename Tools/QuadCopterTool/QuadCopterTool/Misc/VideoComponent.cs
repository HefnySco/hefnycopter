using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Net;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;

namespace QuadCopterTool.Misc.Video
{
    public class VideoComponent
    {
        public delegate void delegate_ImageReceived(MemoryStream memStream);
        protected  bool mVideoCapturing;
        protected byte[] buffer = new byte[1024];
        protected System.Threading.Thread mVideoThread;

        public delegate_ImageReceived mdelegate_ImageReceived;


        #region "Properties"

      
        public bool Running
        {
            get
            {
                return (mVideoCapturing == true);
            }
        }


        #endregion

        /// <summary>
        /// 
        /// </summary>
        /// <param name="MethodImageReceived">delegate to function called when a frame is received.</param>
        public VideoComponent(delegate_ImageReceived MethodImageReceived)
        {
            mdelegate_ImageReceived = new delegate_ImageReceived(MethodImageReceived);
        }

        public void Capture()
        {
            mVideoCapturing = true;
            mVideoThread = new System.Threading.Thread(CaptureFrame);
            mVideoThread.Start();
        }


        public void Close()
        {
            mVideoCapturing = false;
            System.Threading.Thread.Sleep(100);
        }



        protected void CaptureFrame()
        {
            MemoryStream memStream = new MemoryStream();

            int nLastSnapShot = System.Environment.TickCount;
            while (mVideoCapturing)
            {
                try
                {

                    HttpWebRequest httpRequest = (HttpWebRequest)WebRequest.Create(Properties.Settings.Default["VideoConnection"].ToString());
                    httpRequest.Timeout = 30000;
                    httpRequest.Method = "GET";

                    using (HttpWebResponse httpResponse = (HttpWebResponse)httpRequest.GetResponse())
                    {
                        using (Stream responseStream = httpResponse.GetResponseStream())
                        {
                            int bytesRead;
                            memStream.SetLength(0);
                            while ((bytesRead = responseStream.Read(buffer, 0, buffer.Length)) > 0)
                            {
                                memStream.Write(buffer, 0, bytesRead);
                            }
                            memStream.Seek(0, SeekOrigin.Begin);

                        }
                    }

                    if (memStream.Length > 0)
                    {
                        mdelegate_ImageReceived(memStream);
                    }
                }
                catch
                { }
            }
        }



    }
}
