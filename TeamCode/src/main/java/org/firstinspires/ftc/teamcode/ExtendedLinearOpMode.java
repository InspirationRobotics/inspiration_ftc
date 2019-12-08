package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.opencv.core.Mat;


public abstract class ExtendedLinearOpMode extends LinearOpMode {

    public Robot robot = new Robot();

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
        while(currentDiffX > robot.constants.DIST_SENSOR_THRESHOLD && currentDiffY > robot.constants.DIST_SENSOR_THRESHOLD) {

            currentDiffX = targetX - getHorizontalDistance();
            currentDiffY = targetY - getVerticalDistance();

            double targetAngle = getTargetAngle(targetX, targetY);
            strafe(targetAngle);
        }

        stopMotors();
    }

}
