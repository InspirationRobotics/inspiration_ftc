package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "MoveBlockToFoundation")
public class MoveBlockToFoundation extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);

        sleep(1400);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);

        sleep(750);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);

        sleep(750);


        moveToFoundation(3,1);

        sleep(100);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);

        sleep(1000);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_OPEN);

        sleep(750);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);
    }
}
