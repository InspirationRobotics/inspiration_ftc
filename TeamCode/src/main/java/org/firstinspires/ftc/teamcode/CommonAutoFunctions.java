package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.Math;

/* note: every single encoder procedure takes "lfcurr [...] rbcurr" as parameters
   because it matters from where you access the current number of encoder ticks,
   planning to fix this later
*/

public abstract class CommonAutoFunctions extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public final double COUNTS_PER_MOTOR_REV    = 537.6 ;
    public final double DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
    public final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double ROBOT_DIAMETER = 18;
    public final double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

    public double[] globalCoordinates = {48, 24};
    public double globalHeading = 0;
    
    public void hwit()
    {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();
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

    public void driveToXPos(double tgt,
			    int lfcurr, int rfcurr, int lbcurr, int rbcurr) {
	    if (Math.abs(tgt - globalCoordinates[0]) < 0.2) {
	        return;
	    }
	
	    double tgtHeading = (tgt > globalCoordinates[0]) ? -90 : 90;

	    encoderTurnDuplicate(tgtHeading - globalHeading, 0.2, 10,
			     lfcurr, rfcurr, lbcurr, rbcurr);

	    telemetry.addData("dist", Math.abs(tgt - globalCoordinates[0]));
	    telemetry.update();
	    encoderDriveByInches(Math.abs(tgt - globalCoordinates[0]), 0.2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

	
    }

    public void driveToYPos(double tgt,
			    int lfcurr, int rfcurr, int lbcurr, int rbcurr) {
	    if (Math.abs(tgt - globalCoordinates[1]) < 0.2) {
	        return;
	    }
	
	    double tgtHeading = (tgt > globalCoordinates[1]) ? 0 : 180;

	    encoderTurnDuplicate(tgtHeading - globalHeading, 0.2, 10,
			     lfcurr, rfcurr, lbcurr, rbcurr);
	    encoderDriveByInches(Math.abs(tgt - globalCoordinates[1]), 0.2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

    }
}
