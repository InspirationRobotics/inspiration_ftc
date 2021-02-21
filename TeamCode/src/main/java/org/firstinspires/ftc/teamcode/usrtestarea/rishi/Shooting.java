package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="Shooting", group="SimpleAuto")
public class Shooting extends CommonAutoFunctions {

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


        waitForStart();

        robot.shooterOne.setVelocity(-182, AngleUnit.DEGREES);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1500);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1500);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1500);
        robot.shooter.setPosition(0.6);
        sleep(500);
        robot.shooter.setPosition(0.2);
        sleep(1000);
        robot.shooterOne.setVelocity(0);

    }
}
