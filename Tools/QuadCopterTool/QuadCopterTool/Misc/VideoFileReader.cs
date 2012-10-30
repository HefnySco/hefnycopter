using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;


namespace QuadCopterTool.Misc.Video
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
            if (mVideoFile.Length == 0) throw new BadImageFormatException();
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

           // VideoFileIndex v = GetFrameOffsetByTickCount(100);
        }

        public VideoFileIndex GetFrameOffsetByTickCount(int TickCount)
        {
            int BinaryIndex = 0, MaxIndex = mDataFrameList.Count, MinIndex=0;
            while (BinaryIndex < mDataFrameList.Count)
            {
            
                if ((mDataFrameList.Count - BinaryIndex) == 1)
                {
                    return mDataFrameList[BinaryIndex];
                }
                

                if (mDataFrameList[BinaryIndex].FrameTickCount > TickCount) 
                {
                    MaxIndex = BinaryIndex;
                    BinaryIndex = (MinIndex + MaxIndex) / 2;

                    continue;
                }
                if (mDataFrameList[BinaryIndex + 1].FrameTickCount < TickCount)
                {
                    MinIndex = BinaryIndex;
                    BinaryIndex = (MinIndex + MaxIndex) / 2;
                    
                    continue;
                }

                return mDataFrameList[BinaryIndex];
                
            }
            return null;
        }


        public MemoryStream GetFrameByOffset(VideoFileIndex oVideoFileIndex)
        {
            MemoryStream ImageStream = new MemoryStream();
            mVideoFile.Seek(oVideoFileIndex.FramePosition, SeekOrigin.Begin);
            byte[] imageArray = new byte[oVideoFileIndex.FrameLength];
            mVideoFile.Read(imageArray, 0,oVideoFileIndex.FrameLength);
            ImageStream.Write(imageArray, 0, imageArray.Length);
            return ImageStream;
        }
    }
}
