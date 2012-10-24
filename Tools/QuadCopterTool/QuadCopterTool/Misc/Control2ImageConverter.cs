/*
 * 
 * 
 *  see: http://www.dreamincode.net/code/snippet4326.htm 
 */
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;


namespace QuadCopterTool.Misc
{
    public class Control2ImageConverter
    {
        #region ImageHandling

        /// <summary>
        /// Convert any control to a PngBitmapEncoder
        /// </summary>
        /// <param name="controlToConvert">The control to convert to an ImageSource</param>
        /// <returns>The returned ImageSource of the controlToConvert</returns>
        private static PngBitmapEncoder getImageFromControl(Control controlToConvert)
        {
            // save current canvas transform
            Transform transform = controlToConvert.LayoutTransform;

            // get size of control
            Size sizeOfControl = new Size(controlToConvert.ActualWidth, controlToConvert.ActualHeight);
            // measure and arrange the control
            controlToConvert.Measure(sizeOfControl);
            // arrange the surface
            controlToConvert.Arrange(new Rect(sizeOfControl));

            // craete and render surface and push bitmap to it
            RenderTargetBitmap renderBitmap = new RenderTargetBitmap((Int32)sizeOfControl.Width, (Int32)sizeOfControl.Height, 96d, 96d, PixelFormats.Pbgra32);
            // now render surface to bitmap
            renderBitmap.Render(controlToConvert);
            
            // encode png data
            PngBitmapEncoder pngEncoder = new PngBitmapEncoder();
            // puch rendered bitmap into it
            pngEncoder.Frames.Add(BitmapFrame.Create(renderBitmap));

            // return encoder
            return pngEncoder;
        }

        /// <summary>
        /// Get an ImageSource of a control
        /// </summary>
        /// <param name="controlToConvert">The control to convert to an ImageSource</param>
        /// <returns>The returned ImageSource of the controlToConvert</returns>
        public static ImageSource GetImageOfControl(Control controlToConvert)
        {
            // return first frame of image
            return getImageFromControl(controlToConvert).Frames[0];
        }

        /// <summary>
        /// Save an image of a control
        /// </summary>
        /// <param name="controlToConvert">The control to convert to an ImageSource</param>
        ///  /// <param name="fileName">The location to save the image to</param>
        /// <returns>The returned ImageSource of the controlToConvert</returns>
        public static Boolean SaveImageOfControl(Control controlToConvert, String fileName)
        {
            try
            {
                // create a file stream for saving image
                using (System.IO.FileStream outStream = new System.IO.FileStream(fileName, System.IO.FileMode.Create))
                {
                    // save encoded data to stream
                    getImageFromControl(controlToConvert).Save(outStream);
                }
            }
            catch (Exception e)
            {
                // display for debugging
                Console.WriteLine("Exception caught saving stream: {0}", e.Message);
                // return fail
                return false;
            }

            // return that passed
            return true;
        }

        #endregion
    }
}
