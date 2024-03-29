package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Autonomous(name="Auto Collect Rings Quick", group="SimpleAuto")
public class AutoCollectRingsQuick extends CommonAutoFunctions {

    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* init hardware */
        hwit();
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();

        //cvinit();

        sleep(2000);

        double[] wobbleGoalPos = {24, 84};
//        if (pipeline.rings == SkystoneDeterminationPipeline.NumberOfRings.four)
//        {
//            wobbleGoalPos[0] = 24;
//            wobbleGoalPos[1] = 132;
//        } else if (pipeline.rings == SkystoneDeterminationPipeline.NumberOfRings.one) {
//            wobbleGoalPos[0] = 48;
//            wobbleGoalPos[1] = 108;
//        }
//
//        while (!isStarted()) {
//            telemetry.addData("ringnum", pipeline.rings);
//            telemetry.update();
//        }

        waitForStart();

        encoderDriveByInchesVel(12, 3, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());


        /*encoderTurnDuplicateVel(-30, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        encoderDriveByInchesVel(12, 3, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        encoderTurnDuplicateVel(-globalHeading, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        driveToYPos(73, 3,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        driveToXPos(38, 3,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        encoderTurnDuplicateVel(-globalHeading, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());*/

        robot.shooterOne.setVelocity(-210, AngleUnit.DEGREES);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooterOne.setVelocity(0);

        //turn around to get the ring

        /*encoderTurnDuplicateVel(globalHeading, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());*/

        robot.collector.setPower(1);

        driveToYPos(47, 3,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        sleep(1000);

        robot.collector.setPower(1);

        driveToYPos(72, 3,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        encoderTurnDuplicateVel(-globalHeading, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

//        int count = 1;
//
//        while (count <= 166) {
//            robot.collector.setPower(1);
//            sleep(20);
//            robot.collector.setPower(0);
//            sleep(20);
//
//            count = count +1;
//        }

        robot.shooterOne.setVelocity(-177, AngleUnit.DEGREES);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);

//        driveToYPos(wobbleGoalPos[1], 3,
//                robot.frontLeft.getCurrentPosition(),
//                robot.frontRight.getCurrentPosition(),
//                robot.backLeft.getCurrentPosition(),
//                robot.backRight.getCurrentPosition());
//
//        driveToXPos(wobbleGoalPos[0], 3,
//                robot.frontLeft.getCurrentPosition(),
//                robot.frontRight.getCurrentPosition(),
//                robot.backLeft.getCurrentPosition(),
//                robot.backRight.getCurrentPosition());
//
//        encoderTurnDuplicateVel(-globalHeading - 135, 2, 10,
//                robot.frontLeft.getCurrentPosition(),
//                robot.frontRight.getCurrentPosition(),
//                robot.backLeft.getCurrentPosition(),
//                robot.backRight.getCurrentPosition());

            telemetry.addData("Heading", globalHeading);
            telemetry.addData("Status", globalCoordinates[0]);
            telemetry.addData("Status", globalCoordinates[1]);
            telemetry.update();

        }
    }
