package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="MoveArm")
public class MoveArm extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {


        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initDetector();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //collect skystone
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_MID);
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
//            robot.frontPivot.setPosition(0.8);
//            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
//            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
//            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
//            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
//            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);

            //move to foundation

        }

    }

}