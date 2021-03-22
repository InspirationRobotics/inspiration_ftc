package org.firstinspires.ftc.teamcode.usrtestarea.shruti;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class RingCVCode extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Amount of Rings", pipeline.rings);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        public enum NumberOfRings
        {
            four, one, zero
        }

        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final double FOUR_RING_THRESHOLD = (double) 0.6;
//        final int ONE_RING_THRESHOLD = 135;
        final double ONE_RING_THRESHOLD = (double) 0.15;


        Point topLeft = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point bottomRight = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point bottomLeft = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y+ REGION_HEIGHT);
        Point topRight = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        double avg1;

        public volatile NumberOfRings rings = NumberOfRings.four;
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
           // region1_Cb = Cb.submat(new Rect(ratio));
        }

        public Mat processFrame(Mat input)
        {

            inputToCb(input);

            double x = Math.abs(topRight.x - bottomLeft.x);
            double y = Math.abs(topRight.y - bottomLeft.y);

            double ratio = (double) Core.mean(new Rect(y, x)).val[0];

            //avg1 = (double) Core.mean(ratio).val[0];

            Imgproc.rectangle (input, topLeft, bottomRight, RED, 2);

            rings = NumberOfRings.four;
            if(ratio > FOUR_RING_THRESHOLD){
                rings = NumberOfRings.four;
            }else if (ratio > ONE_RING_THRESHOLD){
                rings = NumberOfRings.one;
            }else{
                rings = NumberOfRings.zero;
            }

            Imgproc.rectangle( input, topLeft, bottomRight, GREEN, -1);

            return input;
        }
    }
}
