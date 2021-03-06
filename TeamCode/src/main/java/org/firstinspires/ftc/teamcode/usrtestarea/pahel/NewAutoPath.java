package org.firstinspires.ftc.teamcode.usrtestarea.pahel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

//Right now it just has the ring collecting part. After we collect both rings using the new method, we can continue the wobble goal stuff the same way that we do in the other autonomous path because the robot should be in the same position.

@Autonomous(name="Auto Collect Rings", group="SimpleAuto")
public class NewAutoPath extends CommonAutoFunctions {

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

        //second ring
        robot.collector.setPower(1);


        driveToXYPos(47, 35.5, 3,
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

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

        sleep(1000);


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
