//package org.firstinspires.ftc.teamcode.usrtestarea.rishi;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvSwitchableWebcam;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name = "Ring")
//public class RingDetector extends LinearOpMode {
////    OpenCvInternalCamera phoneCam;
//    OpenCvInternalCamera webcam;
//    SkystoneDeterminationPipeline pipeline;
//
//    public void runOpMode()
//    {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//
//            public void onOpened()
//            {
//                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Amount of Rings", pipeline.rings);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(100);
//        }
//    }
//
//    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
//    {
//        public enum NumberOfRings
//        {
//            four, one, zero
//        }
//
//        Mat hsv = new Mat();
//        Mat thresholded_orange = new Mat();
//        Size size = new Size(352, 198);
//        Rect roi = new Rect(0, 50, 352, 148);
//
//        List<MatOfPoint> contours;
//        Rect bounding_rect = new Rect();
//
//        double largest_area;
//
//
//        public volatile NumberOfRings rings = NumberOfRings.zero;
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            Imgproc.resize(input, input, size);
//            input = new Mat(input.clone(), roi);
//
//            /* color conversion and thresholding */
//            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV, 3);
//            Core.inRange(hsv, new Scalar(15, 100, 40), new Scalar(35, 255, 255), thresholded_orange);
//
//            /* locate all the contours */
//            contours = new ArrayList<>();
//            Imgproc.findContours(thresholded_orange, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            /* get the largest contour area, turn it into a bounding box */
//            largest_area = 0;
//            if (!contours.isEmpty()) {
//
//                for(int i = 0; i < contours.size(); i++) /* iterate through the contours */
//                {
//                    double area = Imgproc.contourArea(contours.get(i));  /* get contour area */
//                    if( area > largest_area )
//                    {
//                        largest_area = area;
//                        /* get a bounding rectangle based on the largest contour */
//                        bounding_rect = Imgproc.boundingRect(contours.get(i));
//                    }
//                }
//
//                /* draw the bounding rect if it is large enough */
//                if (largest_area > 100)
//                    Imgproc.rectangle(input, bounding_rect.tl(), bounding_rect.br(), new Scalar(0, 255, 0), 3);
//
//            }
//
//            if (bounding_rect.width / bounding_rect.height > 2.5) {
//                rings = NumberOfRings.one;
//            } else if (largest_area < 150) {
//                rings = NumberOfRings.zero;
//            }
//            else {
//                rings = NumberOfRings.four;
//            }
//
//            return input;
//        }
//    }
//}
