package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CommonAutoFunctions extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public final double COUNTS_PER_MOTOR_REV    = 537.6 ;
    public final double DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
    public final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double ROBOT_DIAMETER = 18;
    public final double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

    public void encoderDriveByInches(double leftIn, double rightIn, double speed, int timeoutSec) {
        int lTgt;
        int rTgt;

        if (opModeIsActive()) {
            lTgt = (int) (leftIn * COUNTS_PER_INCH);
            rTgt = (int) (rightIn * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(robot.frontLeft.getTargetPosition() + lTgt);
            robot.backLeft.setTargetPosition(robot.backLeft.getTargetPosition() + lTgt); // will this redundancy harm?
            robot.frontRight.setTargetPosition(robot.frontRight.getTargetPosition() + rTgt);
            robot.backRight.setTargetPosition(robot.backRight.getTargetPosition() + rTgt);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.frontLeft.setPower(speed);
            robot.backLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backRight.setPower(speed);

            while ((opModeIsActive() && runtime.seconds() < timeoutSec) &&
                    (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())) {}

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

    public void encoderTurn(int degrees, double speed, int timeoutSec)
    {
        encoderDriveByInches((degrees / 360) * (ROBOT_CIRCUMFERENCE), -(degrees / 360) * (ROBOT_CIRCUMFERENCE), speed, timeoutSec);
    }
}