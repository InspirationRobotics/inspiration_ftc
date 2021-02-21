package org.firstinspires.ftc.teamcode.usrtestarea.shruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="Simple CV Auto Storm", group="Simple Auto")
public class integratedCVAuto extends CommonAutoFunctions {
    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    /* versions that take current encoder position as a parameter due to the weird encoder jumping thing */
    public void encoderDriveByInchesDuplicate(double leftIn, double rightIn, double speed, int timeoutSec, int lfcurr, int rfcurr,
                                              int lbcurr, int rbcurr) {


        if (opModeIsActive()) {

            int lTgt = (int) (leftIn * 23);
            int rTgt = (int) (rightIn * 23);

            robot.frontLeft.setTargetPosition(lfcurr + lTgt);
            robot.backLeft.setTargetPosition(lbcurr + lTgt); // will this redundancy harm?
            robot.frontRight.setTargetPosition(rfcurr + rTgt);
            robot.backRight.setTargetPosition(rbcurr + rTgt);

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
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
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
    public void encoderTurnDuplicate(int degrees, double speed, int timeoutSec)
    {
        encoderDriveByInchesDuplicate((degrees / 360) * (ROBOT_CIRCUMFERENCE), -(degrees / 360) * (ROBOT_CIRCUMFERENCE), speed, timeoutSec, robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
    }

    @Override
    public void runOpMode() {

        /* init hw */
        robot.frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        robot.frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        robot.backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        robot.backRight = hardwareMap.get(DcMotorEx.class, "rb");
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("start:", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
        telemetry.addData("tgt", "%7d, %7d, %7d, %7d", robot.frontLeft.getCurrentPosition() + (int)(24 * 21.37), robot.frontRight.getCurrentPosition() + (int)(24 * 21.37), robot.backLeft.getCurrentPosition() + (int)(24 * 21.37), robot.backRight.getCurrentPosition() + (int)(24 * 21.37));
        telemetry.update();
        waitForStart();



        encoderDriveByInchesDuplicate(24, 24, 0.2, 5, robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());

    }
}
