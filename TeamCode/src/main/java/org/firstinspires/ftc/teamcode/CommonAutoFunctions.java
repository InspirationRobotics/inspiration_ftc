package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

/* note: every single encoder procedure takes "lfcurr [...] rbcurr" as parameters
   because it matters from where you access the current number of encoder ticks,
   planning to fix this later
*/

public abstract class CommonAutoFunctions extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public OpenCvInternalCamera phoneCam;
    public SkystoneDeterminationPipeline pipeline;

    public final double COUNTS_PER_MOTOR_REV = 537.6;
    public final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
    public final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double ROBOT_DIAMETER = 18;
    public final double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

    public double[] globalCoordinates = {51, 18};
    public double globalHeading = 0;
    public double imuStart;

    public void hwit()
    {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();
    }

    public void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = robot.hwmap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
    }

    public void imuStart() {
        imuStart = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void cvinit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.pauseViewport();

//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            public void onOpened() {
//                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//        });
        phoneCam.openCameraDevice();
        phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

    public void closeCamera() {
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
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

        encoderDriveByInchesForTurns(ROBOT_CIRCUMFERENCE * (degrees / 360) * 1.72, -ROBOT_CIRCUMFERENCE * (degrees / 360) * 1.72,
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

        encoderDriveByInchesForTurnsVel(ROBOT_CIRCUMFERENCE * (degrees / 360) * 1.72, -ROBOT_CIRCUMFERENCE * (degrees / 360) * 1.72,
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

        if (dispY > 0) {
            tgtHeading = -(Math.toDegrees(Math.atan(dispY / dispX)));
        } else {
            tgtHeading = (dispX < 0) ? 90 + (Math.toDegrees(Math.atan(dispY / dispX))) : -90 - (Math.toDegrees(Math.atan(dispY / dispX)));
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

    public double unNormalizeHeading(double h) {
        return (h < 0) ? -h : h;
    }

//    public void imuTurn(double tgtDeg, double speed) {
//        int direction;
//        tgtDeg = sumAndNormalizeHeading(imuStart, tgtDeg);
//
////        if ((tgtDeg >= 0 &&
////                robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0)
////            || (tgtDeg < 0 &&
////                robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0)) {
////
////            direction = (tgtDeg > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)
////                    ? 1 : -1;
////        } else {
////
////        }
//
//        if (Math.abs(tgtDeg - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 180) {
//            direction = (tgtDeg > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)
//                    ? 1 : -1;
//        } else {
//            direction = (tgtDeg > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)
//                    ? -1 : 1;
//        }
//
//        while (!(
//                (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < (tgtDeg + 1))
//                && (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > (tgtDeg - 1))))
//        {
//            robot.frontLeft.setPower(speed * -direction);
//            robot.backLeft.setPower(speed * -direction);
//            robot.frontRight.setPower(speed * direction);
//            robot.backRight.setPower(speed * direction);
//            telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
//            telemetry.addData("tgt", tgtDeg);
//            telemetry.update();
//        }
//    }

    public void imuTurn2(double tgtDeg, double speed, double imuStart) {

        int directionality;

        globalHeading = sumAndNormalizeHeading(globalHeading, -tgtDeg);

        tgtDeg = tgtDeg + imuStart;
        if (tgtDeg <= -180) {
            tgtDeg = tgtDeg + 360;
        } else if (tgtDeg > 180) {
            tgtDeg = tgtDeg - 360;
        }

        if (tgtDeg > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) {
            if (Math.abs(tgtDeg - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 180) {
                directionality = -1;
            } else {
                directionality = 1;
            }
        } else if (tgtDeg < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) {
            if (Math.abs(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - tgtDeg) > 180) {
                directionality = 1;
            } else {
                directionality = -1;
            }
        } else {
            return;
        }

        while (!(
                (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < (tgtDeg + 1))
                        && (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > (tgtDeg - 1))) && opModeIsActive())
        {
            robot.frontLeft.setPower(speed * -directionality);
            robot.backLeft.setPower(speed * -directionality);
            robot.frontRight.setPower(speed * directionality);
            robot.backRight.setPower(speed * directionality);
            telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.addData("imu start", imuStart);
            telemetry.addData("tgt", tgtDeg);
            telemetry.addData("directionality", directionality);
            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
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
