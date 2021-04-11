package org.firstinspires.ftc.teamcode.AutonomousRegionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CommonAutoRegionals;
import org.firstinspires.ftc.teamcode.NewRobot;

/* uses the old position tracking system as a backup in case something goes wrong with the other odometry one */

@Autonomous(name="Auto Old Pos Track", group="Regionals")
public class RegionalAutoOldPosTrack extends CommonAutoRegionals {

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

        cvinit();

        sleep(3000);

        while (!isStarted()) {
            telemetry.addData("ringnum", pipeline.returnNum());
            telemetry.update();
        }

        newrobot.servoWobbleGoal.setPosition(-0.6);

        waitForStart();

        sleep(1000);
        numberOfRings = pipeline.returnNum();
        // override 1
//        numberOfRings = 1;
//        wobbleGoalPos[0] = 48;
//        wobbleGoalPos[1] = 108;

        double[] wobbleGoalPos = {24, 84};
        if (numberOfRings == 0) {
            wobbleGoalPos[0] = 24;
            wobbleGoalPos[1] = 84;
        } else if (numberOfRings == 4) {
            wobbleGoalPos[0] = 24;
            wobbleGoalPos[1] = 132;
        } else if (numberOfRings == 1) {
            wobbleGoalPos[0] = 48;
            wobbleGoalPos[1] = 120;
        }
        closeCamera();

        telemetry.addLine("started");

        stop();

    }

}