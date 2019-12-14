package org.firstinspires.ftc.teamcode;
import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.opencv.core.Mat;


public abstract class ExtendedLinearOpMode extends LinearOpMode {

    public Robot robot = new Robot();
    public SkystoneDetector detector = new SkystoneDetector();
    private ElapsedTime runtime = new ElapsedTime();


    public void initDetector() {
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
    }

    public void disableDetector() {
        detector.disable();
    }

    public boolean skyStoneIsVisible(String side) {
        boolean visible = false;

        if (detector.isVerifiedSkystone(side)) {
            visible = true;
        }

        else if (!detector.isVerifiedSkystone(side)) {
            visible = false;
        }

        return visible;
    }

    public void setPower (double left_power, double right_power) {

        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(left_power);
        robot.leftBack.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightBack.setPower(right_power);
    }

    public void initPuddle(HardwareMap hwmp) {

        /** Status: in use
         * Usage: (hardwareMap passed through to initialize robot in one simple function, rather than calling three seperate ones)
         */

        robot.setHardwareMap(hwmp);
        robot.initParaguayFoundationMover();
        robot.initTilter();
        robot.initDrivebase();
    }

    public void timedDrive(Direction direction, long timeMS, double speed) {

        /** Status: in use
         * Usage: Drive the drive base motors by time with the ability to pick direction and speed
         */

        long startTime = System.currentTimeMillis();
        long endTime = startTime + timeMS;

        while (System.currentTimeMillis() < endTime) {

            if (direction == Direction.LEFT) {
                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(-speed);
            }

            else if (direction == Direction.RIGHT) {
                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.rightBack.setPower(speed);
            }

            else if (direction == Direction.FORWARD) {
                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(speed);
            }

            else if (direction == Direction.BACKWARD) {
                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.rightBack.setPower(-speed);
            }
        }

        stopMotors();

    }

    public void timedMotor(DcMotor inputMotor, long timeMS, double speed) {

        /** Status: in use
         * Usage: Move an input motor for a certain time and speed
         */

        long startTime = System.currentTimeMillis();
        long endTime = startTime + timeMS;

        while (System.currentTimeMillis() < endTime) {
            inputMotor.setPower(speed);
        }

        inputMotor.setPower(0);
    }


    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

    }

    public double getDistance(DistanceSensor inputDistSensor, DistanceUnit distanceUnit) {
        double returnDist = inputDistSensor.getDistance(distanceUnit);

        return returnDist;
    }

    public double getHorizontalDistance() {

        double horoizontalDist;

        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.CM) + robot.constants.LEFT_DIST_BIAS;
        double rightDistance = 358.14 - robot.distanceRight.getDistance(DistanceUnit.CM) - robot.constants.RIGHT_DIST_BIAS;

        if(robot.distanceLeft.getDistance(DistanceUnit.CM) > 245) {
            horoizontalDist = rightDistance;
        }

        else {
            horoizontalDist = leftDistance;
        }

        return horoizontalDist;
    }

    public double getVerticalDistance() {

        double verticalDistance;

        double backDistance = robot.distanceBack.getDistance(DistanceUnit.CM) + robot.constants.BACK_DIST_BIAS;
        double frontDistance = 358.14 - robot.distanceFront.getDistance(DistanceUnit.CM) - robot.constants.FRONT_DIST_BIAS;

        if(robot.distanceFront.getDistance(DistanceUnit.CM) > 245) {
            verticalDistance = backDistance;
        }

        else {
            verticalDistance = frontDistance;
        }

        return verticalDistance;
    }

    public double getTargetAngle(double targetX, double targetY) {

        double targetAngle;
        double cpX = getHorizontalDistance();
        double cpY = getVerticalDistance();

        double diffX = targetX - cpX;
        double diffY = targetY - cpY;

        double targetAngleRad = Math.atan2((diffY + 0.0000001), (diffX + 0.0000001));
        targetAngle = (180*targetAngleRad)/(Math.PI);

        if(diffX < 0 && diffY < 0) {
            targetAngle = targetAngle + 180;
        }
        else if (diffX > 0 && diffY < 0) {
            targetAngle = targetAngle + 180;
        }

        return targetAngle;
    }

    public void strafe(double angle) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double powerLeftFront = Range.clip((0.70710678)*((Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerLeftBack = Range.clip((0.70710678)*((-Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerRightFront = Range.clip((0.70710678)*((-Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerRightBack = Range.clip((0.70710678)*((Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);

    }

    public void moveToPos(double targetX, double targetY) {
        double startDiffX = targetX - getHorizontalDistance();
        double startDiffY = targetY - getVerticalDistance();

        double currentDiffX = startDiffX;
        double currentDiffY = startDiffY;

        while(currentDiffX > robot.constants.DIST_SENSOR_THRESHOLD || currentDiffY > robot.constants.DIST_SENSOR_THRESHOLD) {

            currentDiffX = targetX - getHorizontalDistance();
            currentDiffY = targetY - getVerticalDistance();

            double targetAngle = getTargetAngle(targetX, targetY);
            strafe(targetAngle);

            telemetry.addData("X Pos", getHorizontalDistance());
            telemetry.addData("Y Pos", getVerticalDistance());
            telemetry.addData("Diff X", currentDiffX);
            telemetry.addData("Diff Y", currentDiffY);
        }

        stopMotors();
    }

    public void wallAlign(double targetDist, Direction direction, DistanceSensor inputDistance, double maintainedAngle, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        if(direction == Direction.FORWARD) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(90, maintainedAngle);
                }

                else {
                    strafeGyro(270, maintainedAngle);
                }

            }
        }

        else if(direction == Direction.BACKWARD) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(270, maintainedAngle);
                }

                else {
                    strafeGyro(90, maintainedAngle);
                }

            }
        }

        else if(direction == Direction.LEFT) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(180, maintainedAngle);
                }

                else {
                    strafeGyro(0, maintainedAngle);
                }

            }
        }

        else if(direction == Direction.RIGHT) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(0, maintainedAngle);
                }

                else {
                    strafeGyro(180, maintainedAngle);
                }

            }
        }

        stopMotors();

    }

    public void gyroTurn(double targetAngle, double speed, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        while (opModeIsActive() && (System.currentTimeMillis() < endTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, targetAngle, robot.constants.P_TURN_COEFF);
            telemetry.update();
        }

        stopMotors();
    }

    public boolean onHeading(double speed, double angle, double PCoeff) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        robot.rightFront.setPower(leftSpeed);
        robot.rightBack.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void strafeGyro(double angle, double maintainedAngle) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double powerLeftFront = Range.clip((0.70710678)*((Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerLeftBack = Range.clip((0.70710678)*((-Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerRightFront = Range.clip((0.70710678)*((-Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);
        double powerRightBack = Range.clip((0.70710678)*((Math.toDegrees(Math.sin(angle)))+(Math.toDegrees(Math.cos(angle)))), -1, 1);

        powerLeftBack = 0.9*powerLeftBack;
        powerLeftFront = 0.9*powerLeftFront;
        powerRightBack = 0.9*powerRightBack;
        powerRightFront = 0.9*powerRightFront;

        double error = getError(maintainedAngle);
        double error_proportioned = Range.clip((error*robot.constants.P_TURN_COEFF), -0.1, 1);

        powerLeftBack = powerLeftBack - error_proportioned;
        powerLeftFront = powerLeftFront - error_proportioned;
        powerRightBack = powerRightBack + error_proportioned;
        powerRightFront = powerRightFront + error_proportioned;

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);
    }

    public void encoderDrive(double left_in, double right_in, double speed, double timeoutS) {

        int left_distanceEnc = (int) (robot.constants.COUNTS_PER_INCH * left_in);
        int right_distanceEnc =  (int) (robot.constants.COUNTS_PER_INCH * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);

            setTargetPosition(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(Math.abs(speed), Math.abs(speed));

            while ((opModeIsActive() && runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", left_in);
                telemetry.addData("Target Distance Right (in)", right_in);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            stopMotors();

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }

    }

    public void setMotorRunMode(DcMotor.RunMode runmode) {
        robot.leftFront.setMode(runmode);
        robot.rightFront.setMode(runmode);
        robot.leftBack.setMode(runmode);
        robot.rightBack.setMode(runmode);
    }

    public void setTargetPosition(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + rightTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);

    }

    public void extend(double speed, double timeS) {
        runtime.reset();

        while(opModeIsActive() && (timeS > runtime.seconds())) {
            robot.extension.setPower(speed);
        }

        robot.extension.setPower(0);
    }

    public void intake(double speed, double timeS, boolean useDistanceSensor) {
        runtime.reset();

        if(!useDistanceSensor) {
            while (opModeIsActive() && (timeS > runtime.seconds())) {
                robot.intake.setPower(speed);
            }
        }

        else {
            while (opModeIsActive() && (timeS > runtime.seconds()) && (robot.intakeDistance.getDistance(DistanceUnit.INCH) > 3)) {
                robot.intake.setPower(speed);
            }
        }
        robot.intake.setPower(0);
    }


    public void lift(double speed, double timeS) {
        runtime.reset();

        while(opModeIsActive() && (timeS > runtime.seconds())) {
            robot.leftLift.setPower(speed);
            robot.rightLift.setPower(speed);
        }

        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);
    }

}
