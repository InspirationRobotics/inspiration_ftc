package org.firstinspires.ftc.teamcode.AutonomousRegionals.test;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonAutoRegionals;
import org.firstinspires.ftc.teamcode.NewRobot;

public class ShootingTest extends CommonAutoRegionals {
    NewRobot newrobot = new NewRobot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* init hardware */
        hwit();
        newrobot.setHardwareMap(hardwareMap);
        newrobot.initDrivetrain();
        newrobot.initAllServos();
        newrobot.initMiscMotors();
        initIMU();
        sleep(2000);
        imuStart();

        int numberOfRings;

        newrobot.servoWobbleGoal.setPosition(-0.6);

        waitForStart();

        newrobot.shooter.setPosition(0.6);
        sleep(1000);
        newrobot.shooter.setPosition(0.2);
        sleep(1000);
        newrobot.shooter.setPosition(0.6);
        sleep(1000);
        newrobot.shooter.setPosition(0.2);
        sleep(1000);
        newrobot.shooter.setPosition(0.6);
        sleep(1000);
        newrobot.shooter.setPosition(0.2);
        sleep(1000);
        newrobot.shooter.setPosition(0.6);

    }
}
