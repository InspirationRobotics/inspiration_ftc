package org.firstinspires.ftc.teamcode.CV;

import com.inspiration.inspcv.OpenCVPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public Mat processFrame(Mat rgba, Mat gray) {

        Size size = new Size(198, 352);
        Imgproc.resize(rgba, rgba, size);

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
        Core.inRange(grey, new Scalar(10), new Scalar(50), thresholded);
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
                    if (working_bounding_rect.tl().y >= bounding_rect_gold.tl().y /* && working_bounding_rect.br().y <= bounding_rect_gold.br().y */) {
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

    public boolean isVerifiedSkystone() {
        if (Math.abs(bounding_rect_gold_global.br().y - bounding_rect_global.br().y) < 30 && (bounding_rect_global.br().y != 0d && bounding_rect_gold_global.br().y != 0d)) {
            if (Math.abs(bounding_rect_gold_global.tl().y - bounding_rect_global.tl().y) < 90 && (bounding_rect_global.tl().y != 0d && bounding_rect_gold_global.tl().y != 0d))
            return true;
        }
        return false;
    }

    public boolean skystoneIsInPlace() {
        /* ... and then check to see whether the edge of the bounding rect is within the given parameters, and is the correct size */
        if (this.isVerifiedSkystone() && bounding_rect_global.width > 300 && (bounding_rect_global.x < 300 && bounding_rect_global.x > 200) /* make these constant */) {
            return true;
        }
        return false;
    }

    public boolean robotInPosition() {
        if (isVerifiedSkystone() && skystoneIsInPlace()) {
            return true;
        }
        return false;
    }

    public double[] returnCoords() {
        double[] coords = {bounding_rect_global.br().y, bounding_rect_gold_global.br().y};
        return coords;
    }

}
