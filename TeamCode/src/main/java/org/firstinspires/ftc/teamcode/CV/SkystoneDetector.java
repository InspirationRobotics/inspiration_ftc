package org.firstinspires.ftc.teamcode.CV;

import com.inspiration.inspcv.OpenCVPipeline;

import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by rishi on 2019-10-17
 */

public class SkystoneDetector extends OpenCVPipeline {
    
    private boolean showContours = true;

    /* bounding rect and contours */
    Rect bounding_rect_global = new Rect();
    private List<MatOfPoint> contours = new ArrayList<>();
    Rect bounding_rect_gold_global = new Rect();
    private List<MatOfPoint> contours_gold = new ArrayList<>();
    private Rect roi = new Rect(59, 0, 234, 198);

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public Mat processFrame(Mat rgba, Mat gray) {

        Size size = new Size(352, 198);
        Imgproc.resize(rgba, rgba, size);
        rgba = new Mat(rgba.clone(), roi);

        /* bounding boxes */
        Rect bounding_rect = new Rect();
        Rect bounding_rect_gold = new Rect();
        Rect working_bounding_rect = new Rect();

        /* matricies: hsv, thresholded, and rgba/thresholded cropped */
        Mat hsv = new Mat();
        Mat grey = new Mat();
        Mat thresholded = new Mat();
        Mat thresholded_gold = new Mat();

        /* change colorspace */
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        /* threshold, blur, and erode */
        Core.inRange(hsv, new Scalar(10, 180, 130), new Scalar(25, 255, 255), thresholded_gold);
//        Imgproc.blur(thresholded_gold, thresholded_gold, new Size(15, 15));
//        Imgproc.erode(thresholded_gold, thresholded_gold, new Mat(30, 30, 0));

        /* find contours */
        contours_gold = new ArrayList<>();
        Imgproc.findContours(thresholded_gold, contours_gold, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        /* create a bounding rect based on the largest contour */

        if (showContours && !contours_gold.isEmpty()) {

            double largest_area = 0;
            for(int i = 0; i < contours_gold.size(); i++ ) /* iterate through the contours */
            {
                double area = Imgproc.contourArea(contours_gold.get(i));  /* get contour area */
                if( area > largest_area )
                {
                    largest_area = area; /* save the largest contour area */

                    /* get a bounding rectangle based on the largest contour */
                    bounding_rect_gold = Imgproc.boundingRect(contours_gold.get(i));
                }
            }

            /* draw the contours and the bounding rect */
            Imgproc.drawContours(rgba, contours_gold, -1, new Scalar(255, 255, 0), 1, 8);

        }

        /* draw a bounding box around black */

        /* change the colorspace */
        Imgproc.cvtColor(rgba, grey, Imgproc.COLOR_RGB2GRAY, 3);


        /* threshold, blur, erode, and dilate */
        Core.inRange(grey, new Scalar(0), new Scalar(50), thresholded);
//        Imgproc.blur(thresholded, thresholded, new Size(15, 15));
        Imgproc.erode(thresholded, thresholded, new Mat(15, 15, 0));
//        Imgproc.dilate(thresholded, thresholded, new Mat(15, 15, 0));

        /* find contours */
        contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        /* create a bounding rect based on the largest contour */

        if (showContours && !contours.isEmpty()) {

            double largest_area = 0;

            for(int i = 0; i < contours.size(); i++ ) /* iterate through the contours */
            {
                double area = Imgproc.contourArea(contours.get(i));  /* get contour area */
                if ( area > largest_area )
                {
                    /* save the largest contour area */

                    /* get a bounding rectangle based on the largest contour */
                    working_bounding_rect = Imgproc.boundingRect(contours.get(i));
                    if (working_bounding_rect.area() > 300 && working_bounding_rect.tl().y >= bounding_rect_gold.tl().y && working_bounding_rect.br().y <= bounding_rect_gold.br().y) {
                        largest_area = area;
                        bounding_rect = working_bounding_rect;
                    }
                }
            }

            /* draw the contours and the bounding rect */
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 1, 8);
        }

        thresholded.release();

        if (bounding_rect_gold != null && bounding_rect != null) {
            Imgproc.rectangle(rgba, bounding_rect.tl(), bounding_rect.br(), new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(rgba, bounding_rect_gold.tl(), bounding_rect_gold.br(), new Scalar(255, 255, 0), 3);
        }

        bounding_rect_global = bounding_rect;
        bounding_rect_gold_global = bounding_rect_gold;


        hsv.release();
        thresholded_gold.release();
        grey.release();



        /* return the rgba matrix */
        return rgba;
    }

    public boolean isVerifiedSkystone(String side) {
        if (Math.abs(bounding_rect_gold_global.br().y - bounding_rect_global.br().y) < 30 && (bounding_rect_global.br().y != 0d && bounding_rect_gold_global.br().y != 0d)) {
            if (Math.abs(bounding_rect_gold_global.tl().y - bounding_rect_global.tl().y) < 90 && (bounding_rect_global.tl().y != 0d && bounding_rect_gold_global.tl().y != 0d))
                if (side == "blue")
                    if (bounding_rect_gold_global.br().x < bounding_rect_global.br().x)
                        return true;
                else if (side == "red")
                    if (bounding_rect_gold_global.br().x > bounding_rect_global.br().x)
                        return true;

            return true;
        }
        return false;
    }

    public int skystoneId(String side) {
        if(side == "blue") {
            if (bounding_rect_global.br().x <= 234 && bounding_rect_global.br().x >= 157) {
                return 3;
            } else if (bounding_rect_global.br().x <= 156 && bounding_rect_global.br().x >= 79) {
                return 2;
            } else if (bounding_rect_global.br().x <= 78 && bounding_rect_global.br().x >= 1) {
                return 1;
            } else {
                return 0;
            }
        }
    }
    public double[] returnCoords() {
        double[] coords = {bounding_rect_global.br().x, bounding_rect_gold_global.br().x};
        return coords;
    }

}
