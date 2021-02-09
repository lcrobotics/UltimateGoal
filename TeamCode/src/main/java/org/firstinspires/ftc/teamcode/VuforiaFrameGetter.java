package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.BlockingQueue;

// VuforiaFrameGetter gets RGB values from the webcam
// and can efficiently sum values within a rectangle thanks to integral imaging.
// It can also use these sums to find the goalposts.
public class VuforiaFrameGetter {

    // Pixel values are retrieved from here...
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = null;
    // ...and stored in here: indexed by channel, then x-coordinate, then y-coordinate
    // (0=red, 1=green, 2=blue)
    public int[][][] rgbValues = null;
    // Stores the integral image values:
    // the sum of all values in a given channel inside a rectangle starting at (0, 0)
    public int[][][] integralImg = null;
    // Image dimensions: 800x448 in our case, assumed not to change!
    public int imgWidth = 0;
    public int imgHeight = 0;
    // Store position of the goal (top left corner) to be retrieved by other classes
    public int xMax = -1, yMax = -1;

    // Constructor receives BlockingQueue<VuforiaLocalizer.CloseableFrame> object from vuforia
    // (see VuforiaSuperOp for use)
    public VuforiaFrameGetter(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue){
        this.frameQueue = frameQueue;
    }

    // Store pixel values retrieved from frameQueue in rgbValues
    // and calculate the integral image
    public void updateFrame(){

        // There are a number of unimportant intermediate stages
        // which begin here...
        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = frameQueue.take();
            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);
                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    Bitmap bmp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bmp.copyPixelsFromBuffer(rgb.getPixels());
                    // ...and end here

                    // Runs when the first frame is received:
                    // find the image size and initialize the arrays accordingly
                    // Note that the integral image is 1 pixel taller and wider
                    // since it includes not only the sums of rectangles of width/height 0
                    // but also those as wide/tall as the entire image
                    if (rgbValues == null) {
                        imgWidth = bmp.getWidth();
                        imgHeight = bmp.getHeight();
                        rgbValues = new int[3][imgWidth][imgHeight];
                        integralImg = new int[3][imgWidth+1][imgHeight+1];
                    }

                    // Set rgbValues
                    for (int x = 0; x < imgWidth; x++) {
                        for (int y = 0; y < imgHeight; y++) {

                            int pixel = bmp.getPixel(x, y);
                            int red = Color.red(pixel);
                            int green = Color.green(pixel);
                            int blue = Color.blue(pixel);

                            rgbValues[0][x][y] = red;
                            rgbValues[1][x][y] = green;
                            rgbValues[2][x][y] = blue;

                            // Integral image needs to be initialized to original
                            integralImg[0][x+1][y+1] = red;
                            integralImg[1][x+1][y+1] = green;
                            integralImg[2][x+1][y+1] = blue;
                        }
                    }

                    // Calculate the integral image
                    for (int x = 1; x < imgWidth; x++) {
                        for (int y = 1; y < imgHeight+1; y++) {
                            integralImg[0][x+1][y] += integralImg[0][x][y];
                            integralImg[1][x+1][y] += integralImg[1][x][y];
                            integralImg[2][x+1][y] += integralImg[2][x][y];
                        }
                    }

                    for (int x = 1; x < imgWidth+1; x++) {
                        for (int y = 1; y < imgHeight; y++) {
                            integralImg[0][x][y+1] += integralImg[0][x][y];
                            integralImg[1][x][y+1] += integralImg[1][x][y];
                            integralImg[2][x][y+1] += integralImg[2][x][y];
                        }
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Return the sum of the values in channel c of the image
    public int sumOfRect(int c, int x, int y, int w, int h) {
        if (integralImg == null) {
            return 0;
        }
        return
                +integralImg[c][x+w][y+h]
                        -integralImg[c][x][y+h]
                        -integralImg[c][x+w][y]
                        +integralImg[c][x][y];
    }

    // Find the coordinates of (the top-left corner of)
    // the rectangle of a given width/height
    // whose value sum in a given channel is maximized
    // and store them in xMax and yMax
    public void updateMaxRect(int c, int w, int h) {
        xMax = -1;
        yMax = -1;
        int sMax = -1;
        for (int x = 0; x <= imgWidth-w; x++) {
            for (int y = 0; y <= imgHeight-h; y++) {
                int s = sumOfRect(c, x, y, w, h);
                if (s > sMax) {
                    xMax = x;
                    yMax = y;
                    sMax = s;
                }
            }
        }
    }
}
