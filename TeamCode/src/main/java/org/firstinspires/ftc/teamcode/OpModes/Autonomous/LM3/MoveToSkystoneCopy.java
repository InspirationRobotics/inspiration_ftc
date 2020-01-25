package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name="MoveToSkystone copy")
public class MoveToSkystoneCopy extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        moveToSkystoneCopy(1, 0.8);
        //grabAutoArm();

        sleep(100);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);

        sleep(1400);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);

        sleep(750);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);

        sleep(1000);
        //collect skystone

        //move to foundation

    }

}