package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="SimpleAutoNonPowerShot", group="SimpleAuto")
public class SimpleAutoNonPowerShot extends CommonAutoFunctions {

    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* init hw */
        hwit();
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();

        /* relative (inches) to the corner closest to the starting position
            x, y, angle to turn at the end
        */
        int[][] dropzonePositions = {
                {24, 80, 60},
                {24, 104, -60},
                {24, 128, 60}
        };
        /* a: 0; b: 1; c: 2 */
        int dropzoneNum = 0;

        int[][] ringsPositions = {
                {36, 66},
        };

        int parkY = 84;

        telemetry.addData("Status", "ready");
        telemetry.update();
        waitForStart();

        /* wobble goal */
        encoderDriveByInchesDuplicate(dropzonePositions[dropzoneNum][1] - 18, dropzonePositions[dropzoneNum][1] - 18, 0.5, 5,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        encoderTurnDuplicate(dropzonePositions[dropzoneNum][2], 0.5, 5,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        encoderTurnDuplicate(-dropzonePositions[dropzoneNum][2], 0.5, 5,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        /* ring 1 */

        encoderDriveByInchesDuplicate(ringsPositions[0][1] - dropzonePositions[dropzoneNum][1],
                ringsPositions[0][1] - dropzonePositions[dropzoneNum][1],
                0.5,
                8,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        encoderTurnDuplicate(-90, 0.5, 5,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        encoderDriveByInchesDuplicate(ringsPositions[0][0] - dropzonePositions[dropzoneNum][0],
                ringsPositions[0][0] - dropzonePositions[dropzoneNum][0],
                0.5,
                8,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        encoderTurnDuplicate(90, 0.5, 5,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        robot.shooterOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double motorVel = -1470;
        robot.shooterOne.setVelocity(motorVel);
        runtime.reset();
        for (int i = 0; i < 3; ++i) {
            robot.shooter.setPosition(0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
            }
            robot.shooter.setPosition(0.6);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
            }
        }

        /* park */

        encoderDriveByInchesDuplicate(parkY - ringsPositions[0][1],
                parkY - ringsPositions[0][1],
                0.5,
                8,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

    }
}
