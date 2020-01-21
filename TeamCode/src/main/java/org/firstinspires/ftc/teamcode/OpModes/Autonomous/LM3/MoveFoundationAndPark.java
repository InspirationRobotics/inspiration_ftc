package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "MoveFoundationAndPark")
public class MoveFoundationAndPark extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);

        sleep(250);

        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        sleep(750);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);

        sleep(500);

        moveFoundation();

        sleep(250);

        parkBridge();

    }
}
