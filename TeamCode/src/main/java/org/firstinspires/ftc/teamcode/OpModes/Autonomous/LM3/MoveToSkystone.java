package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="MoveToSkystone")
public class MoveToSkystone extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();


        moveToSkystone(3, 0.125);
        grabAutoArm();

        //collect skystone
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_MID);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);

        //move to foundation
        encoderDrive(50,50,0.8, 0.8, 5000);

        wallAlign(0.6, 18, robot.distanceRight, Direction.RIGHT, 5000);
        wallAlign(0.6, 30, robot.distanceFront, Direction.FORWARD, 5000);
    }

}