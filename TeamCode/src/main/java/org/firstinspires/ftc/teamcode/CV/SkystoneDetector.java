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

    private byte detection = 0; /* detection status */
    private boolean showContours = true;

    /* bounding rect and contours */
    Rect bounding_rect;
    private List<MatOfPoint> contours = new ArrayList<>();
    Rect bounding_rect_gold;
    private List<MatOfPoint> contours_gold = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public Mat processFrame(Mat rgba, Mat gray) {

        /* matricies: hsv, thresholded, and rgba/thresholded cropped */
        Mat hsv = new Mat();
        Mat thresholded = new Mat();
        Mat thresholded_gold = new Mat();

        /* kernel for dilation and roi for cropping */
        Mat kernel = new Mat(30, 30, 0);

        /* draw a bounding box around black */

        /* change the colorspace */
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        /* threshold, blur, and dilate */
        Core.inRange(hsv, new Scalar(0, 0, 0), new Scalar(255, 255, 10), thresholded);
        Imgproc.blur(thresholded, thresholded, new Size(15, 15));
        Imgproc.dilate(thresholded, thresholded, kernel);

        /* find contours */
        contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        /* create a bounding rect based on the largest contour */

        if (showContours && !contours.isEmpty()) {

            double largest_area = 0;
            for(int i = 0; i < contours.size(); i++ ) /* iterate through the contours */
            {
                double area = Imgproc.contourArea(contours.get(i));  /* get contour area */
                if( area > largest_area )
                {
                    largest_area = area; /* save the largest contour area */

                    /* get a bounding rectangle based on the largest contour */
                    bounding_rect = Imgproc.boundingRect(contours.get(i));
                }
            }

            /* draw the contours and the bounding rect */
            Imgproc.drawContours(rgba, contours, -1, new Scalar(255, 255, 0), 1, 8);
            Imgproc.rectangle(rgba, bounding_rect.tl(), bounding_rect.br(), new Scalar(0, 255, 0), 3);
        }

        thresholded.release();

        /* draw a bounding box around yellow */

        /* threshold, blur, and dilate */
        Core.inRange(hsv, new Scalar(20, 0, 0), new Scalar(255, 255, 255), thresholded_gold);
        Imgproc.blur(thresholded_gold, thresholded_gold, new Size(15, 15));
        Imgproc.dilate(thresholded_gold, thresholded_gold, kernel);

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
            Imgproc.rectangle(rgba, bounding_rect_gold.tl(), bounding_rect.br(), new Scalar(0, 0, 0), 3);
        }

        hsv.release();
        kernel.release();
        thresholded_gold.release();

        /* return the rgba matrix */
        return rgba;
    }

    public byte getDetectionStatus() {
        /* check whether the bounding box exists */
        if (bounding_rect != null) {
            /* ... and then check to see whether the edge of the bounding rect is within the given parameters, and is the correct size */
            if (bounding_rect.width > 300 && (bounding_rect.x < 300 && bounding_rect.x > 200) /* make these constant */) {
                detection = 1;
            } else { detection = 0; }
        }
        return detection;
    }
}
