package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.usrtestarea.shruti.RingCVCode;
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

import java.lang.Math;

/* note: every single encoder procedure takes "lfcurr [...] rbcurr" as parameters
   because it matters from where you access the current number of encoder ticks,
   planning to fix this later
*/

public abstract class CommonAutoFunctions extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public OpenCvInternalCamera phoneCam;
    public SkystoneDeterminationPipeline pipeline;

    public final double COUNTS_PER_MOTOR_REV    = 537.6 ;
    public final double DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
    public final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double ROBOT_DIAMETER = 18;
    public final double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

    public double[] globalCoordinates = {47, 18};
    public double globalHeading = 0;
    
    public void hwit()
    {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();
    }

    public void cvinit() {
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
    }

    public void encoderDriveByInches(double distance, double speed, int timeoutSec, int lfcurr, int rfcurr,
                                              int lbcurr, int rbcurr) {

        if (opModeIsActive()) {
	    
            int speedMult = (distance >= 0) ? 1 : -1;
	    
            int tgt = (int) (distance * 43);

            robot.frontLeft.setTargetPosition(lfcurr + tgt);
            robot.backLeft.setTargetPosition(lbcurr + tgt);
            robot.frontRight.setTargetPosition(rfcurr + tgt);
            robot.backRight.setTargetPosition(rbcurr + tgt);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.frontLeft.setPower(speed * speedMult);
            robot.backLeft.setPower(speed * speedMult);
            robot.frontRight.setPower(speed * speedMult);
            robot.backRight.setPower(speed * speedMult);

            while ((opModeIsActive() && runtime.seconds() < timeoutSec) &&
                    (robot.frontLeft.isBusy() && robot.backLeft.isBusy()) || (robot.frontRight.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("lf, rf, lb, rb", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("tgt:", "%7d, %7d, %7d, %7d", robot.frontLeft.getTargetPosition(), robot.frontRight.getTargetPosition(),
                        robot.backLeft.getTargetPosition(), robot.backRight.getTargetPosition());
                telemetry.update();
            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
	
	    globalCoordinates = calculateTargetPosition(globalHeading, distance);
    }

    public void encoderDriveByInchesVel(double distance, double vel, int timeoutSec, int lfcurr, int rfcurr,
                                     int lbcurr, int rbcurr) {

        if (opModeIsActive()) {

            int speedMult = (distance >= 0) ? 1 : -1;

            int tgt = (int) (distance * 43);

            robot.frontLeft.setTargetPosition(lfcurr + tgt);
            robot.backLeft.setTargetPosition(lbcurr + tgt);
            robot.frontRight.setTargetPosition(rfcurr + tgt);
            robot.backRight.setTargetPosition(rbcurr + tgt);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.frontLeft.setVelocity(vel * speedMult, AngleUnit.RADIANS);
            robot.backLeft.setVelocity(vel * speedMult, AngleUnit.RADIANS);
            robot.frontRight.setVelocity(vel * speedMult, AngleUnit.RADIANS);
            robot.backRight.setVelocity(vel * speedMult, AngleUnit.RADIANS);

            while ((opModeIsActive() && runtime.seconds() < timeoutSec) &&
                    (robot.frontLeft.isBusy() && robot.backLeft.isBusy()) || (robot.frontRight.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("lf, rf, lb, rb", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("tgt:", "%7d, %7d, %7d, %7d", robot.frontLeft.getTargetPosition(), robot.frontRight.getTargetPosition(),
                        robot.backLeft.getTargetPosition(), robot.backRight.getTargetPosition());
                telemetry.update();
            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        globalCoordinates = calculateTargetPosition(globalHeading, distance);
    }

    
    public void encoderDriveByInchesForTurns(double leftIn, double rightIn, double speed, int timeoutSec, int lfcurr, int rfcurr,
                                              int lbcurr, int rbcurr) {

	

        if (opModeIsActive()) {

            int lSpeedMult = (leftIn >= 0) ? 1 : -1;
            int rSpeedMult = (rightIn >= 0) ? 1 : -1;

            int lTgt = (int) (leftIn * 45);
            int rTgt = (int) (rightIn * 45);

            robot.frontLeft.setTargetPosition(lfcurr + lTgt);
            robot.backLeft.setTargetPosition(lbcurr + lTgt);
            robot.frontRight.setTargetPosition(rfcurr + rTgt);
            robot.backRight.setTargetPosition(rbcurr + rTgt);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.frontLeft.setPower(speed * lSpeedMult);
            robot.backLeft.setPower(speed * lSpeedMult);
            robot.frontRight.setPower(speed * rSpeedMult);
            robot.backRight.setPower(speed * rSpeedMult);

            double tickDifference;
            double proportionalConstant = 0.01;
            while ((opModeIsActive() && runtime.seconds() < timeoutSec) &&
                    (robot.frontLeft.isBusy() && robot.backLeft.isBusy()) || (robot.frontRight.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("lf, rf, lb, rb", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("tgt:", "%7d, %7d, %7d, %7d", robot.frontLeft.getTargetPosition(), robot.frontRight.getTargetPosition(),
                        robot.backLeft.getTargetPosition(), robot.backRight.getTargetPosition());
                telemetry.update();
                tickDifference = (robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition()) * proportionalConstant;

                if (tickDifference > 0.1) {
                    robot.frontLeft.setPower(tickDifference * lSpeedMult);
                    robot.backLeft.setPower(tickDifference * lSpeedMult);
                    robot.frontRight.setPower(tickDifference * rSpeedMult);
                    robot.backRight.setPower(tickDifference * rSpeedMult);
                } else {
                    robot.frontLeft.setPower(0.1 * lSpeedMult);
                    robot.backLeft.setPower(0.1 * lSpeedMult);
                    robot.frontRight.setPower(0.1 * rSpeedMult);
                    robot.backRight.setPower(0.1 * rSpeedMult);
                }
            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
  
    }
    public void encoderDriveByInchesForTurnsVel(double leftIn, double rightIn, double vel, int timeoutSec, int lfcurr, int rfcurr,
                                             int lbcurr, int rbcurr) {



        if (opModeIsActive()) {

            int lSpeedMult = (leftIn >= 0) ? 1 : -1;
            int rSpeedMult = (rightIn >= 0) ? 1 : -1;

            int lTgt = (int) (leftIn * 45);
            int rTgt = (int) (rightIn * 45);

            robot.frontLeft.setTargetPosition(lfcurr + lTgt);
            robot.backLeft.setTargetPosition(lbcurr + lTgt);
            robot.frontRight.setTargetPosition(rfcurr + rTgt);
            robot.backRight.setTargetPosition(rbcurr + rTgt);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.frontLeft.setVelocity(vel * lSpeedMult, AngleUnit.RADIANS);
            robot.backLeft.setVelocity(vel * lSpeedMult, AngleUnit.RADIANS);
            robot.frontRight.setVelocity(vel * rSpeedMult, AngleUnit.RADIANS);
            robot.backRight.setVelocity(vel * rSpeedMult, AngleUnit.RADIANS);

            double tickDifference;
            double proportionalConstant = 0.01;
            while ((opModeIsActive() && runtime.seconds() < timeoutSec) &&
                    (robot.frontLeft.isBusy() && robot.backLeft.isBusy()) || (robot.frontRight.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("lf, rf, lb, rb", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("tgt:", "%7d, %7d, %7d, %7d", robot.frontLeft.getTargetPosition(), robot.frontRight.getTargetPosition(),
                        robot.backLeft.getTargetPosition(), robot.backRight.getTargetPosition());
                telemetry.update();
                tickDifference = (robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition()) * proportionalConstant * lSpeedMult;

                if (tickDifference > 0.1) {
                    robot.frontLeft.setPower(tickDifference * lSpeedMult);
                    robot.backLeft.setPower(tickDifference * lSpeedMult);
                    robot.frontRight.setPower(tickDifference * rSpeedMult);
                    robot.backRight.setPower(tickDifference * rSpeedMult);
                } else {
                    robot.frontLeft.setPower(0.1 * lSpeedMult);
                    robot.backLeft.setPower(0.1 * lSpeedMult);
                    robot.frontRight.setPower(0.1 * rSpeedMult);
                    robot.backRight.setPower(0.1 * rSpeedMult);
                }
            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void encoderTurnDuplicate(double degrees, double speed, int timeoutSec,
                                     int lfcurr, int rfcurr, int lbcurr, int rbcurr) {

        globalHeading = sumAndNormalizeHeading(globalHeading, degrees);

	    encoderDriveByInchesForTurns(ROBOT_CIRCUMFERENCE * (degrees/360) * 1.72, -ROBOT_CIRCUMFERENCE * (degrees/360) * 1.72,
                speed,
                timeoutSec,
                lfcurr,
                rfcurr,
                lbcurr,
                rbcurr
                );
    }

    public void encoderTurnDuplicateVel(double degrees, double vel, int timeoutSec,
                                     int lfcurr, int rfcurr, int lbcurr, int rbcurr) {

        globalHeading = sumAndNormalizeHeading(globalHeading, degrees);

        encoderDriveByInchesForTurnsVel(ROBOT_CIRCUMFERENCE * (degrees/360) * 1.72, -ROBOT_CIRCUMFERENCE * (degrees/360) * 1.72,
                vel,
                timeoutSec,
                lfcurr,
                rfcurr,
                lbcurr,
                rbcurr
        );
    }

    public double sumAndNormalizeHeading(double initialHeading, double addedHeading) {
        double headingSum = (initialHeading + addedHeading);

        if (headingSum > 180) {
            headingSum = headingSum - 360;
        } else if (headingSum <= -180) {
            headingSum = headingSum + 360;
        }
        return headingSum;
    }

    public double[] calculateTargetPosition(double heading, double distance) {
	    double[] tempArray = globalCoordinates;

	    double globalHeadingRad = Math.toRadians(globalHeading);

	    tempArray[0] = -(distance * Math.sin(globalHeadingRad)) + tempArray[0];
	    tempArray[1] = (distance * Math.cos(globalHeadingRad)) + tempArray[1];

	    return tempArray;
    }

    public void driveToXPos(double tgt, double speed,
			    int lfcurr, int rfcurr, int lbcurr, int rbcurr) {
	    if (Math.abs(tgt - globalCoordinates[0]) < 0.2) {
	        return;
	    }
	
	    double tgtHeading = (tgt > globalCoordinates[0]) ? -90 : 90;

	    encoderTurnDuplicateVel(tgtHeading - globalHeading, 2, 10,
			     lfcurr, rfcurr, lbcurr, rbcurr);

	    telemetry.addData("dist", Math.abs(tgt - globalCoordinates[0]));
	    telemetry.update();
	    encoderDriveByInchesVel(Math.abs(tgt - globalCoordinates[0]), speed, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

	
    }

    public void driveToXYPos(double tgtX, double tgtY, double speed,
                            int lfcurr, int rfcurr, int lbcurr, int rbcurr) {

        /* not accurate as of now */

        if (Math.abs(tgtX - globalCoordinates[0]) < 0.2 &&
                Math.abs(tgtY - globalCoordinates[1]) < 0.2) {
            return;
        }

        double dispX = tgtX - globalCoordinates[0];
        double dispY = tgtY - globalCoordinates[1];
        double tgtHeading;

        if(dispY > 0) {
            tgtHeading = -(Math.toDegrees(Math.atan(dispY/dispX)));
        } else {
            tgtHeading = (dispX < 0) ? 90 + (Math.toDegrees(Math.atan(dispY/dispX))) : -90 - (Math.toDegrees(Math.atan(dispY/dispX)));
        }
        encoderTurnDuplicateVel(tgtHeading - globalHeading, 2, 10,
                lfcurr, rfcurr, lbcurr, rbcurr);

        encoderDriveByInchesVel(Math.sqrt(Math.pow(dispX, 2) + Math.pow(dispY, 2)), speed, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());


    }

    public void driveToYPos(double tgt, double speed,
			    int lfcurr, int rfcurr, int lbcurr, int rbcurr) {
	    if (Math.abs(tgt - globalCoordinates[1]) < 0.2) {
	        return;
	    }
	
	    double tgtHeading = (tgt > globalCoordinates[1]) ? 0 : 180;

	    encoderTurnDuplicateVel(tgtHeading - globalHeading, 2, 10,
			     lfcurr, rfcurr, lbcurr, rbcurr);
	    encoderDriveByInchesVel(Math.abs(tgt - globalCoordinates[1]), speed, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

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

        final int FOUR_RING_THRESHOLD = 180;
        final int ONE_RING_THRESHOLD = 135;

        Point topLeft = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point bottomRight = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        public volatile SkystoneDeterminationPipeline.NumberOfRings rings = SkystoneDeterminationPipeline.NumberOfRings.four;
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));
        }

        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle (input, topLeft, bottomRight, RED, 2);

            rings = SkystoneDeterminationPipeline.NumberOfRings.four;
            if(avg1 > FOUR_RING_THRESHOLD){
                rings = SkystoneDeterminationPipeline.NumberOfRings.four;
            }else if (avg1 > ONE_RING_THRESHOLD){
                rings = SkystoneDeterminationPipeline.NumberOfRings.one;
            }else{
                rings = SkystoneDeterminationPipeline.NumberOfRings.zero;
            }

            Imgproc.rectangle( input, topLeft, bottomRight, GREEN, -1);

            return input;
        }


        }
}
