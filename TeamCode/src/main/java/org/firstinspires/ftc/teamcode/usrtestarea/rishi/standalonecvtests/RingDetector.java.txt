import java.util.ArrayList;
import java.io.IOException;
import java.util.List;

import org.opencv.core.*;

import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


public class RingDetector {
    static{ System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) throws IOException, InterruptedException {
        VideoCapture camera = new VideoCapture(2);
        int index = 0;

        /* camera capture matrix, hsv matrix, threshold, region of interest for cropping */
        Mat mat = new Mat();
        Mat hsv = new Mat();
        Mat thresholded_orange = new Mat();
        Rect roi = new Rect(0, 50, 352, 148);

        List<MatOfPoint> contours;
        Rect bounding_rect = new Rect();

        int numberOfRings = 0;
        double largest_area;


        if (!camera.isOpened()) {
            System.out.println("camera cannot be opened, exiting");
            return;
        }

        while (true) {
            if (camera.isOpened()) {
                while (true) {
                    camera.read(mat);

                    /* resize and crop image */
                    Size size = new Size(352, 198);
                    Imgproc.resize(mat, mat, size);
                    mat = new Mat(mat.clone(), roi);

                    /* color conversion and thresholding */
                    Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_BGR2HSV, 3);
                    Core.inRange(hsv, new Scalar(15, 100, 40), new Scalar(35, 255, 255), thresholded_orange);

                    /* locate all the contours */
                    contours = new ArrayList<>();
                    Imgproc.findContours(thresholded_orange, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                    /* get the largest contour area, turn it into a bounding box */
                    largest_area = 0;
                    if (!contours.isEmpty()) {

                        for(int i = 0; i < contours.size(); i++ ) /* iterate through the contours */
                        {
                            double area = Imgproc.contourArea(contours.get(i));  /* get contour area */
                            if( area > largest_area )
                            {
                                largest_area = area;
                                /* get a bounding rectangle based on the largest contour */
                                bounding_rect = Imgproc.boundingRect(contours.get(i));
                            }
                        }

                        /* draw the bounding rect if it is large enough */
                        if (largest_area > 100)
                            Imgproc.rectangle(mat, bounding_rect.tl(), bounding_rect.br(), new Scalar(0, 255, 0), 3);

                    }


                    /* criteria to determine number of rings */
                    if (bounding_rect.width / bounding_rect.height > 2.5) {
                        numberOfRings = 1;
                    } else if (largest_area < 150) {
                        numberOfRings = 0;
                    }
                    else {
                        numberOfRings = 4;
                    }

                    System.out.println(numberOfRings);
                    Imgproc.putText(mat, String.valueOf(numberOfRings), new Point(338, 140), 0, 0.5, new Scalar(0, 255, 0), 2);

                    /* output to the gui */
                    HighGui.imshow("test", mat);
                    index = HighGui.waitKey(1);
                    if (index == 27) {
                        break;
                    }
                }
            }
        }
    }
}