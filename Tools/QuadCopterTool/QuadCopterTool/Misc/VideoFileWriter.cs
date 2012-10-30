using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace QuadCopterTool.Misc.Video
{
    public class VideoFileWriter
    {

        #region "Attributes"

        protected FileStream mVideoFile;
        protected string mFileName;
        protected Int32 mFramesCount;
        
        #endregion


        #region "Properties"

        public Int32 FramesCount
        {
            get
            {
                return mFramesCount;
            }
        }

        #endregion 


        #region "Constructor"

        public VideoFileWriter(string FileName)
        {
            mFileName = FileName;

        }

        /// <summary>
        /// Create -or truncate if exist- video file.
        /// </summary>
        public void Create()
        {
            mVideoFile = System.IO.File.Create(mFileName);
        }

        public void Close()
        {
            mVideoFile.Close();
        }



        public void AddFrame(MemoryStream ImageStream)
        {
            byte[] NumberData = new byte[4];
            NumberData = BitConverter.GetBytes(System.Environment.TickCount);  // save tick count.
            mVideoFile.Write(NumberData, 0, 4);
            mFramesCount += 1;
            NumberData = BitConverter.GetBytes(ImageStream.Length); // save image size
            mVideoFile.Write(NumberData, 0, 8);
            ImageStream.WriteTo(mVideoFile);
        }

        #endregion

    }
}
