package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public abstract class BasicExtendedLinearOpMode extends LinearOpMode {
    public Robot robot = new Robot();
    public SkystoneDetector detector = new SkystoneDetector();
    private ElapsedTime runtime = new ElapsedTime();

    public DistanceSensor frontDistanceSensor = robot.distanceFrontRight;
    public DistanceSensor rearDistanceSensor = robot.distanceBackLeft;

    /* imu functions */

    public void initIMU(HardwareMap ahawmp) {

        robot.imu = hardwareMap.get(BNO055IMU.class, robot.constants.IMU_NAME);

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("imu heading", getHeading());

    }

    public void strafeGyro(double speed, double maintainedAngle) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double powerLeftBack = 0.8*speed;
        double powerLeftFront = -0.8*speed;
        double powerRightBack = -0.8*speed;
        double powerRightFront = 0.8*speed;

        double error = getError(maintainedAngle);
        double error_proportioned = Range.clip((error*0.08), -0.2, 0.2);

        powerLeftBack = powerLeftBack - error_proportioned;
        powerLeftFront = powerLeftFront - error_proportioned;
        powerRightBack = powerRightBack + error_proportioned;
        powerRightFront = powerRightFront + error_proportioned;

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);
    }

    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return -robot.angles.firstAngle;
    }

    public void gyroTurn(double targetAngle, double speed, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        while (opModeIsActive() && (System.currentTimeMillis() < endTime) && !onHeading(speed, targetAngle, robot.constants.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
            telemetry.update();
        }

        stopMotors();
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.constants.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180 && opModeIsActive())  robotError -= 360;
        while (robotError <= -180 && opModeIsActive()) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        if(error > 0) {
            return Range.clip(error * PCoeff, 0.25, 1);
        } else {
            return Range.clip(error * PCoeff, -1, -0.25);
        }
    }

    /* movement */

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void moveToSkystone(int skystoneId, AllianceSide allianceSide) {

        gyroTurn(0,0.5,1);
        encoderStrafeGyro(robot.constants.WALL_DIST_STONE, 1, 0, 3);
        gyroTurn(0,0.5,2);

        double targetDistance;
        DistanceSensor wallAlignSensor;
        Direction wallAlignDirection;

        if (allianceSide == AllianceSide.BLUE) {
            targetDistance = -(8*skystoneId)+18.5;
        } else {
            if (skystoneId < 5) {
                targetDistance = (8 * skystoneId) - 12;
            } else {
                targetDistance = (8 * 4) - 16;
            }
        }

        encoderDrive(targetDistance, targetDistance, 0.5, 0.5, 3500);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        grabAutoArm();
    }

    /* auto arm */

    public void grabAutoArm(){
        //grab stone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        sleep(700);
        double endTime = System.currentTimeMillis() + 325;
        while (System.currentTimeMillis() < endTime) {
            strafeGyro(1, 0);
        }

        stopMotors();

        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(500);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
        endTime = System.currentTimeMillis() + 325;
        while (System.currentTimeMillis() < endTime) {
            strafeGyro(-1, 0);
        }
    }

    public void releaseAutoArm() {
        //release stone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        sleep(400);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_OPEN);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);
    }

    /* movement functions */

    public void encoderStrafe(double units, double speed) {

        units = units*0.68;
        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            // setPowerStrafe(Math.abs(speed), Math.abs(speed));
            setPower(Math.abs(speed), Math.abs(speed));

            while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", units);
                telemetry.addData("Target Distance Right (in)", units);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }
        }
    }

    public void setPower (double left_power, double right_power) {

        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(-left_power);
        robot.leftBack.setPower(-left_power);
        robot.rightFront.setPower(-right_power);
        robot.rightBack.setPower(-right_power);
    }

    public void setTargetPositionStrafe(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightFront.getCurrentPosition() + leftTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + rightTarget);
        robot.rightFront.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void setMotorRunMode(DcMotor.RunMode runmode) {
        robot.leftFront.setMode(runmode);
        robot.rightFront.setMode(runmode);
        robot.leftBack.setMode(runmode);
        robot.rightBack.setMode(runmode);
    }

    public void encoderDrive(double left_in, double right_in, double speed_l, double speed_r, double timeoutS) {

        int left_distanceEnc = -(int) (robot.constants.COUNTS_PER_INCH * left_in);
        int right_distanceEnc =  -(int) (robot.constants.COUNTS_PER_INCH * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            sleep(100);

            setTargetPosition(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(Math.abs(speed_l), Math.abs(speed_r));

            while ((opModeIsActive() && runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", left_in);
                telemetry.addData("Target Distance Right (in)", right_in);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight05", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            stopMotors();

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            sleep(250);

        }

    }

    public void setTargetPosition(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + rightTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void moveToFoundation(int skystoneId, AllianceSide allianceSide) {

        double targetDriveDist = (120 - (8 * (7 + -skystoneId)));

        if (skystoneId > 4) {
            targetDriveDist = targetDriveDist + 5;
        } else {
            targetDriveDist = targetDriveDist - 3;
        }

        gyroTurn(0,0.5,2);

        switch (allianceSide) {
            case RED:
                targetDriveDist = -(targetDriveDist);
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                sleep(50);
//                double endTimeRed = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeRed) {
//                    strafeGyro(1, 0);
//                }
                encoderStrafeGyro(8,1,0,1.5);
                break;
            case BLUE:
                targetDriveDist = targetDriveDist - 16;
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                sleep(50);
//                double endTimeBlue = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeBlue) {
//                    strafeGyro(1, 0);
//                }
                encoderStrafeGyro(8,1,0,1.5);
                break;
        }

        stopMotors();
//        strafeWallDist(robot.constants.WALL_DIST_FOUNDATION,1,robot.distanceLeft, Direction.LEFT, 4000);
        gyroTurn(0, 0.5, 2);

//        releaseAutoArm();
    }

    public void multipleStones(int skystoneId, AllianceSide allianceSide) {

        double targetDriveDist = -(133.5 - (8 * (7 + -skystoneId)));

        if (skystoneId > 4) {
            targetDriveDist = targetDriveDist + 5;
        } else {
            targetDriveDist = targetDriveDist - 3;
        }

        gyroTurn(0,0.5,2);

        switch (allianceSide) {
            case RED:
//                double endTimeRed = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeRed) {
//                    strafeGyro(-1, 0);
//                }
                encoderStrafeGyro(-11,1,0,3);
                gyroTurn(0,0.5,2);
                targetDriveDist = -(targetDriveDist);
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                break;
            case BLUE:
//                double endTimeBlue = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeBlue) {
//                    strafeGyro(-1, 0);
//                }
                encoderStrafeGyro(-11,1,0,3);
                gyroTurn(0,0.5,2);
                targetDriveDist = targetDriveDist + 16;
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                break;
        }

//        strafeWallDist(robot.constants.WALL_DIST_FOUNDATION,1,robot.distanceLeft, Direction.LEFT, 4000);
        gyroTurn(0, 0.5, 2);
        grabAutoArm();
    }

    public void moveToSkystoneStorm(int skystoneId, AllianceSide allianceSide) {

        gyroTurn(0,0.5,1);
        encoderStrafeGyro(robot.constants.WALL_DIST_STONE, 1, 0, 3);
        gyroTurn(0,0.5,2);

        double targetDistance = 19-(8*skystoneId);

        encoderDrive(targetDistance, targetDistance, 0.7, 0.7, 3500);
        grabAutoArmStorm();
    }

    public void grabAutoArmStorm() {
        long downSleepTimeMS = 500;
        long grabWaitTimeMS = 400;
        robot.autoPivot.setPosition(robot.constants.AUTO_PIVOT_DOWN_POSITION);
        sleep(downSleepTimeMS);
        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_GRAB_POSITION);
        sleep(grabWaitTimeMS);
        robot.autoPivot.setPosition(robot.constants.AUTO_PIVOT_COMPACT_POSITION);
    }

    public void releaseAutoArmStorm() {
        long downSleepTimeMS = 500;
        long grabWaitTimeMS = 400;
        robot.autoPivot.setPosition(robot.constants.AUTO_PIVOT_DOWN_POSITION);
        sleep(downSleepTimeMS);
        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
        sleep(grabWaitTimeMS);
        robot.autoPivot.setPosition(robot.constants.AUTO_PIVOT_COMPACT_POSITION);
    }

    public void compactAutoArmStorm() {
        long downSleepTimeMS = 500;
        robot.autoPivot.setPosition(robot.constants.AUTO_PIVOT_COMPACT_POSITION);
    }

    public void moveToFoundationStorm(int skystoneId) {
        double targetDist = 68 + (8*skystoneId);

        gyroTurn(0,0.5,1);
        encoderDrive(targetDist,targetDist,1,1,5);
        gyroTurn(0,0.5,1);

        releaseAutoArmStorm();
    }

    public void multipleStoneStorm(int skystoneId) {
        double targetDist = 68 + (8*skystoneId);

        gyroTurn(0,0.5,1);
        encoderDrive(-targetDist,-targetDist,1,1,5);
        gyroTurn(0,0.5,1);

        grabAutoArmStorm();
    }

    public void moveFoundation(AllianceSide allianceSide) {

        if (allianceSide == AllianceSide.BLUE) {

            gyroTurn(-90, 0.6, 10);

            encoderDrive(-10, -10, 0.3, 0.3, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

            sleep(1000);

//            encoderDrive(-5, -5, 0.7, 0.7, 10);

            encoderDrive(22, 22, 0.8, 0.8, 10);

            gyroTurn(-180, 1, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            sleep(250);

            encoderDrive(5, 5, 1, 1, 0.8);
        }

        else if (allianceSide == AllianceSide.RED) {

//            while ((robot.distanceLeft.getDistance(DistanceUnit.INCH) > 28.5) && opModeIsActive()) {
//                strafeGyro(-1, 0);
//            }

            gyroTurn(-90, 0.6, 10);

            encoderDrive(-10, -10, 0.3, 0.3, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

            sleep(1000);

//            encoderDrive(-5, -5, 0.7, 0.7, 10);

            encoderDrive(18, 18, 0.8, 0.8, 10);

//            sleep(250);

            gyroTurn(-180, 1, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            sleep(250);

            encoderDrive(5, 5, 1, 1, 0.8);
        }

    }

    public void parkBridge(AllianceSide allianceSide) {

        compactAutoArm();

        double targetHeading;
        double strafeDist = -5;

        if (allianceSide == AllianceSide.BLUE) {
            targetHeading = -178;
        } else {
            targetHeading = 2;
            strafeDist = strafeDist*-1;
        }

        encoderStrafeGyro(strafeDist, 0.5, 0, 3);
        gyroTurn(targetHeading, 0.5, 1);
        encoderDrive(24, 24, 1, 1, 3.5);

}

    public void compactAutoArm(){
        robot.frontPivot.setPosition(0.2);
        robot.backClawCollect.setPosition(0.1);
    }

    public void encoderStrafeGyro(double units, double speed, double maintainedAngle, double timeoutS) {

        units = units;
        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        int speedMultiplier;

        if (units > 0) {
            speedMultiplier=1;
        } else {
            speedMultiplier=-1;
        }
        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            //setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            double err_lb = Math.abs(robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition());
            double err_lf = Math.abs(robot.leftFront.getCurrentPosition() -robot.leftFront.getTargetPosition());
            double err_rb = Math.abs(robot.rightBack.getCurrentPosition() -robot.rightBack.getTargetPosition());
            double err_rf = Math.abs(robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition());


            while (opModeIsActive() && (err_lb > robot.constants.ENCODER_STRAFE_ERR_THRESHOLD) && (err_lf > robot.constants.ENCODER_STRAFE_ERR_THRESHOLD) &&
                    (err_rb > robot.constants.ENCODER_STRAFE_ERR_THRESHOLD) && (err_rf > robot.constants.ENCODER_STRAFE_ERR_THRESHOLD) && (System.currentTimeMillis() < endTime)) {

                err_lb = Math.abs(robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition());
                err_lf = Math.abs(robot.leftFront.getCurrentPosition() -robot.leftFront.getTargetPosition());
                err_rb = Math.abs(robot.rightBack.getCurrentPosition() -robot.rightBack.getTargetPosition());
                err_rf = Math.abs(robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition());

                double powerLeftBack = 0.9*speed*speedMultiplier;
                double powerLeftFront = -0.9*speed*speedMultiplier;
                double powerRightBack = -0.9*speed*speedMultiplier;
                double powerRightFront = 0.9*speed*speedMultiplier;

                double error = getError(maintainedAngle);
                double error_proportioned = Range.clip((error*0.1), -0.1, 0.1);

                powerLeftBack = powerLeftBack - error_proportioned;
                powerLeftFront = powerLeftFront - error_proportioned;
                powerRightBack = powerRightBack + error_proportioned;
                powerRightFront = powerRightFront + error_proportioned;

                robot.leftFront.setPower((powerLeftFront));
                robot.leftBack.setPower((powerLeftBack));
                robot.rightFront.setPower((powerRightFront));
                robot.rightBack.setPower((powerRightBack));

            }
            stopMotors();
        }


    }

}