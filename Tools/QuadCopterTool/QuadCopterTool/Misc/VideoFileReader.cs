using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;


namespace QuadCopterTool.Misc
{
    public class VideoFileReader
    {

        FileStream mVideoFile;
        protected List<VideoFileIndex> mDataFrameList;
        

        public VideoFileReader()
        {
            mDataFrameList = new List<VideoFileIndex>();
        }


        public void OpenFile(string FilePath)
        {
            mDataFrameList.Clear();
            mVideoFile = System.IO.File.OpenRead(FilePath);
            byte[] NumberData = new byte[8];
            while (mVideoFile.Position < mVideoFile.Length)
            {
                VideoFileIndex oVideoFileIndex = new VideoFileIndex();
                mVideoFile.Read(NumberData, (int)0, 4);
                oVideoFileIndex.FrameTickCount = BitConverter.ToInt32(NumberData, 0);  // save tick count.
                mVideoFile.Read(NumberData, 0, 8);
                oVideoFileIndex.FrameLength = BitConverter.ToInt32(NumberData, 0); // save image size
                oVideoFileIndex.FramePosition = (int) mVideoFile.Position;
                //ImageArray = new byte[oVideoFileIndex.FrameLength];
                mVideoFile.Seek(oVideoFileIndex.FrameLength, SeekOrigin.Current);
                mDataFrameList.Add(oVideoFileIndex);
            }

            VideoFileIndex v = GetFrameOffsetByTickCount(100);
        }

        public VideoFileIndex GetFrameOffsetByTickCount(int TickCount)
        {
            int BinaryIndex = 0;
            while (BinaryIndex < mDataFrameList.Count)
            {
            
                if ((mDataFrameList.Count - BinaryIndex) == 1)
                {
                    return mDataFrameList[BinaryIndex];
                }
                
                if ((mDataFrameList[BinaryIndex].FrameTickCount <= TickCount) && ((mDataFrameList[BinaryIndex + 1].FrameTickCount >= TickCount)))
                {
                    return mDataFrameList[BinaryIndex];
                }
                else
                {
                    BinaryIndex = (BinaryIndex + mDataFrameList.Count) / 2;
                }
           
            }
            return null;
        }
    }
}
