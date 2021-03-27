package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="Auto Collect Rings Wobble", group="SimpleAuto")
public class AutoCollectRingsWobble extends CommonAutoFunctions {

    /* Declare OpMode members. */
    Robot robot = new Robot();   // Use a Pushbot's hardware

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* init hardware */
        hwit();
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();

//        cvinit();

        sleep(2000);

        double[] wobbleGoalPos = {24, 84};
//        if (pipeline.rings == SkystoneDeterminationPipeline.NumberOfRings.zero) {
//            wobbleGoalPos[0] = 24;
//            wobbleGoalPos[1] = 84;
//        } else if (pipeline.rings == SkystoneDeterminationPipeline.NumberOfRings.four) {
//            wobbleGoalPos[0] = 24;
//            wobbleGoalPos[1] = 132;
//        } else if (pipeline.rings == SkystoneDeterminationPipeline.NumberOfRings.one) {
//            wobbleGoalPos[0] = 48;
//            wobbleGoalPos[1] = 108;
//        }

        // override 1
        SkystoneDeterminationPipeline.NumberOfRings ringnum = SkystoneDeterminationPipeline.NumberOfRings.one;
        wobbleGoalPos[0] = 48;
        wobbleGoalPos[1] = 108;

//        while (!isStarted()) {
//            telemetry.addData("ringnum", pipeline.rings);
//            telemetry.update();
//        }
//
//        telemetry.addLine("Ready to go!");
//        telemetry.update();
//
//        telemetry.addData("num rings", pipeline.rings);
//        telemetry.update();

        robot.servoWobbleGoal.setPosition(-0.6);

        waitForStart();

        robot.servoWobbleGoal.setPosition(-0.4);

//        robot.shooterOne.setVelocity(-211, AngleUnit.DEGREES);
        robot.shooterOne.setPower(-0.60);
        sleep(1000);
        robot.shooter.setPosition(0.2);

        driveToYPos(45, 4,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());


        encoderTurnDuplicateVel(10, 2, 10,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());


        robot.shooter.setPosition(0.6);
        sleep(1000);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(1000);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);
        sleep(1000);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooter.setPosition(0.6);


        if (ringnum == SkystoneDeterminationPipeline.NumberOfRings.zero) {

            driveToYPos(wobbleGoalPos[1], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            driveToXPos(wobbleGoalPos[0], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

        } else if (ringnum == SkystoneDeterminationPipeline.NumberOfRings.four) {

            driveToYPos(wobbleGoalPos[1], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            driveToXPos(wobbleGoalPos[0], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            encoderTurnDuplicateVel(180, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());


        } else if (ringnum == SkystoneDeterminationPipeline.NumberOfRings.one) {

            encoderTurnDuplicateVel(90, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            encoderDriveByInchesVel(12, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            encoderTurnDuplicateVel(-90, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            robot.collector.setPower(1);

//            encoderTurnDuplicateVel(-90, 2, 10,
//                    robot.frontLeft.getCurrentPosition(),
//                    robot.frontRight.getCurrentPosition(),
//                    robot.backLeft.getCurrentPosition(),
//                    robot.backRight.getCurrentPosition());

            encoderDriveByInchesVel(6, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            sleep(3000);

            robot.collector.setPower(0);

            sleep(1000);

            driveToYPos(70, 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());


            robot.shooterOne.setVelocity(-200, AngleUnit.DEGREES);
            sleep(1000);
            robot.shooter.setPosition(0.2);
            sleep(1000);
            robot.shooter.setPosition(0.6);

            driveToYPos(wobbleGoalPos[1], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            driveToXPos(wobbleGoalPos[0], 3,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());

            // WOBBLE GOAL NOW
            robot.wobbleGoal.setPower(1);
            sleep(2000);
            robot.wobbleGoal.setPower(0);
            robot.servoWobbleGoal.setPosition(1.2);
            sleep(500);
            robot.wobbleGoal.setPower(-1);

//            encoderTurnDuplicateVel(-90, 2, 10,
//                    robot.frontLeft.getCurrentPosition(),
//                    robot.frontRight.getCurrentPosition(),
//                    robot.backLeft.getCurrentPosition(),
//                    robot.backRight.getCurrentPosition());

            encoderDriveByInchesVel(12, 2, 10,
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());


        }

    }

}
