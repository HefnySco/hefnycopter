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


namespace QuadCopterTool.Misc
{
    public class VideoComponent
    {

        public delegate void delegate_ImageReceived(MemoryStream memStream);
        protected bool mVideoCapturing;
        protected byte[] buffer = new byte[1024];
        protected System.Threading.Thread mVideoThread;
        public delegate_ImageReceived mdelegate_ImageReceived;

        public bool Running
        {
            get
            {
                return (mVideoCapturing==true);
            }
        }

        public VideoComponent(delegate_ImageReceived MethodImageReceived)
        {
            mdelegate_ImageReceived = new delegate_ImageReceived(MethodImageReceived);
        }

        public void Run()
        {
            mVideoThread = new System.Threading.Thread(CaptureFrame);
            mVideoThread.Start();
            mVideoCapturing = true;
        }

        public void Close()
        {
            mVideoCapturing = false;
            
            System.Threading.Thread.Sleep(100);
        }


        protected void CaptureFrame()
        {
            //            httpRequest.UserAgent = "Mozilla/5.0 (Windows NT 6.1; rv:12.0) Gecko/20100101 Firefox/12.0";
            //httpRequest.Accept = "image/png,image/*;q=0.8,*/*;q=0.5";
            MemoryStream memStream = new MemoryStream();
            int nLastSnapShot = System.Environment.TickCount;
            while (mVideoCapturing)
            {
                try
                {
                    HttpWebRequest httpRequest = (HttpWebRequest)WebRequest.Create(Properties.Settings.Default["VideoConnection"].ToString());
                    httpRequest.Timeout = 30000;
                    httpRequest.Method = "GET";
            
                    int nDuration = System.Environment.TickCount - nLastSnapShot;
                    if (nDuration < 200) System.Threading.Thread.Sleep(50-nDuration);
                    nLastSnapShot = System.Environment.TickCount - nLastSnapShot;
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



                        
                        //ImageSource ctrlImgSource = Control2ImageConverter.GetImageOfControl(tabItemRealTimeData);


                      
                        //// RenderTargetBitmap ctrl = new RenderTargetBitmap(100, 100, 96d, 96d, PixelFormats.Pbgra32);
                        //// ctrl.Render(C);

                        //// //jPegEnc.Frames.Add(BitmapFrame.Create(ctrl));
                        ////// jPegEnc.Save(VideoFile);


                        //// var group = new DrawingGroup();

                        //// ImageDrawing oImageDrawingCamera = new ImageDrawing(bmpCamera, new Rect(0, 0, bmpCamera.Width, bmpCamera.Height));
                        //// ImageDrawing oImageDrawingControl = new ImageDrawing(ctrl, new Rect(0, 0, 100, 100));

                        //// //oImageDrawingControl.
                        //// group.Children.Clear();
                        //// group.Children.Add(oImageDrawingCamera);
                        //// group.Children.Add(oImageDrawingControl);

                        //imgLive.Dispatcher.Invoke(new delegate_ImgLiveInvokeSource(ImgLiveInvokeSource),bmpCamera);
                        //imgLive.Source = bmpCamera; // new DrawingImage(group); //ctrlImgSource;

                        mdelegate_ImageReceived(memStream);
                        //imgLive.Dispatcher.Invoke(new Action(delegate() { ImgLiveInvokeSource(memStream); }));
                    }
                }
                catch
                { }
            }
        }
    }
}
