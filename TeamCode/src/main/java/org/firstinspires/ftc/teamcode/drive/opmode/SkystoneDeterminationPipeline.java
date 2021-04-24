package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class SkystoneDeterminationPipeline extends OpenCvPipeline {
    private boolean showContours = true;
    int ringnum = 0;
    double safetyWidth = 0.01;
    double safetyHeight = 0.01;
    double ratio = 0;

    /* bounding rect and contours */
    private List<MatOfPoint> contours = new ArrayList<>();
    Rect bounding_rect_orange_global = new Rect();
    private List<MatOfPoint> contours_orange = new ArrayList<>();
    private Rect roi = new Rect(109, 0, 234, 198);

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    double largest_area;

    public Mat processFrame(Mat rgba) {

        Size size = new Size(352, 198);
        Imgproc.resize(rgba, rgba, size);
        rgba = new Mat(rgba.clone(), roi);

        /* bounding boxes */
        Rect bounding_rect_orange = new Rect();

        /* matricies: hsv, thresholded, and rgba/thresholded cropped */
        Mat hsv = new Mat();
        Mat grey = new Mat();
        Mat thresholded_orange = new Mat();

        /* change colorspace */
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        /* threshold */
        Core.inRange(hsv, new Scalar(15, 100, 40), new Scalar(35, 255, 255), thresholded_orange);

        /* find contours */
        contours_orange = new ArrayList<>();
        Imgproc.findContours(thresholded_orange, contours_orange, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        /* create a bounding rect based on the largest contour */

        if (showContours && !contours_orange.isEmpty()) {

            largest_area = 0;
            for (int i = 0; i < contours_orange.size(); i++) /* iterate through the contours */ {
                double area = Imgproc.contourArea(contours_orange.get(i));  /* get contour area */
                if (area > largest_area) {
                    largest_area = area; /* save the largest contour area */

                    /* get a bounding rectangle based on the largest contour */
                    bounding_rect_orange = Imgproc.boundingRect(contours_orange.get(i));
                }
            }

            /* draw the contours and the bounding rect */
            Imgproc.drawContours(rgba, contours_orange, -1, new Scalar(255, 255, 0), 1, 8);

        }


        bounding_rect_orange_global = bounding_rect_orange;


        hsv.release();
        thresholded_orange.release();
        grey.release();

        safetyHeight = (double) bounding_rect_orange_global.height + 0.01;
        safetyWidth = (double) bounding_rect_orange_global.width + 0.01;
        ratio = (safetyWidth / safetyHeight);

        if ((bounding_rect_orange_global.height == 0 || bounding_rect_orange_global.width == 0) && (largest_area < 150)) {
            ringnum = 0;
        } else if (ratio < 1.9) {
            ringnum = 4;
        } else if (ratio > 1.9) {
            ringnum = 1;
        }

//            if (bounding_rect_orange_global.height == 0){
//                return rgba;
//            } else if((double)bounding_rect_orange_global.width / (double)bounding_rect_orange_global.height > 2.5) {
//                ringnum = 1;
//            } else if (largest_area < 150) {
//                ringnum = 0;
//            }
//            else {
//                ringnum = 4;
//            }

        /* return the rgba matrix */
        return rgba;
    }

    public int returnNum() {
        return ringnum;
    }

}