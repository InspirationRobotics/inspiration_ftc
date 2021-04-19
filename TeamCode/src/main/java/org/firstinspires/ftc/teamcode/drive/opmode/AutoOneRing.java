package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "autoPaths")
public class AutoOneRing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** control hub */

        /* distance sensors */
        final Rev2mDistanceSensor lfl = hardwareMap.get(Rev2mDistanceSensor.class, "LFL");
        final Rev2mDistanceSensor lft = hardwareMap.get(Rev2mDistanceSensor.class, "LFT");
        final Rev2mDistanceSensor lbl = hardwareMap.get(Rev2mDistanceSensor.class, "LBL");
        final Rev2mDistanceSensor lbt = hardwareMap.get(Rev2mDistanceSensor.class, "LBT");

        /** expansion hub */

        /* motors */
        final DcMotorEx wobbleGoal = hardwareMap.get(DcMotorEx.class, "wobbleGoal");
        final DcMotorEx shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        final DcMotorEx shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        final DcMotorEx collector = hardwareMap.get(DcMotorEx.class, "collector");

        /* servos */
        final Servo wobbleServo = hardwareMap.get(Servo.class, "servoWobbleGoal");
        final Servo shooterTilt = hardwareMap.get(Servo.class, "shooterTilt");
        final Servo magazine = hardwareMap.get(Servo.class, "magazine");

        /* distance sensors */
        final Rev2mDistanceSensor rbl = hardwareMap.get(Rev2mDistanceSensor.class, "RBL");
        final Rev2mDistanceSensor rbt = hardwareMap.get(Rev2mDistanceSensor.class, "RBT");
        final Rev2mDistanceSensor rfl = hardwareMap.get(Rev2mDistanceSensor.class, "RFL");
        final Rev2mDistanceSensor rft = hardwareMap.get(Rev2mDistanceSensor.class, "RFT");

        double OPEN_POS = 0.1;
        double GRAB_POS = 0.9;

        double OUT_POWER = -0.7;
        double IN_POWER = 0.7;

        wobbleServo.setPosition(GRAB_POS);

        OpenCvInternalCamera phoneCam;
        SkystoneDeterminationPipeline pipeline;
        int numberOfRings;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.pauseViewport();

        phoneCam.openCameraDevice();
        phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);


        while (!isStarted()) {
            telemetry.addData("ringnum", pipeline.returnNum());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        numberOfRings = pipeline.returnNum();

        double[] wobbleGoalPos = {24, 84};
        if (numberOfRings == 0) {
            wobbleGoalPos[0] = 56;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 4) {
            wobbleGoalPos[0] = 104;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 1) {
            wobbleGoalPos[0] = 80;
            wobbleGoalPos[1] = 21;
        }

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        telemetry.addLine("started");

        drive.setPoseEstimate(new Pose2d(0,29.5,0));

        Trajectory toRingStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(28, 28))
                .build();

        Trajectory toDropZoneOne_WobbleGoalOne = drive.trajectoryBuilder(toRingStack.end())
                .splineTo(new Vector2d(wobbleGoalPos[0], wobbleGoalPos[1]), 0)
                .build();

        Trajectory collectWobbleGoal = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalOne.end())
                .lineToLinearHeading(new Pose2d(25.5, 13, Math.toRadians(145)))
                .addDisplacementMarker(8, () -> {
                    wobbleGoal.setPower(OUT_POWER);
                })
                .addDisplacementMarker(40, () -> {
                    wobbleGoal.setPower(0);
                })
                .build();

        Trajectory toDropZoneOne_WobbleGoalTwo = drive.trajectoryBuilder(collectWobbleGoal.end())
                .lineToLinearHeading(new Pose2d(76, 16, Math.toRadians(0)))
                .build();


        drive.followTrajectory(toRingStack);

        shooterOne.setPower(0.5);
        shooterTwo.setPower(0.5);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);

        drive.followTrajectory(toDropZoneOne_WobbleGoalOne);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(collectWobbleGoal);

        wobbleServo.setPosition(GRAB_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(toDropZoneOne_WobbleGoalTwo);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);
    }

    public void moveMotorSec(DcMotorEx motor, double power, long runTime) {
        ElapsedTime elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        while(opModeIsActive() && (elapsedTime.milliseconds() < runTime)) {
            motor.setPower(power);
        }

        motor.setPower(0);
        //I feel like a bug
    }

    public class SkystoneDeterminationPipeline extends OpenCvPipeline {
        private boolean showContours = true;
        int ringnum = 0;

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


            if (bounding_rect_orange_global.height == 0){
                return rgba;
            } else if(bounding_rect_orange_global.width / bounding_rect_orange_global.height > 2.5) {
                ringnum = 1;
            } else if (largest_area < 150) {
                ringnum = 0;
            }
            else {
                ringnum = 4;
            }

            /* return the rgba matrix */
            return rgba;
        }

        public int returnNum() {
            return ringnum;
        }
    }
}

