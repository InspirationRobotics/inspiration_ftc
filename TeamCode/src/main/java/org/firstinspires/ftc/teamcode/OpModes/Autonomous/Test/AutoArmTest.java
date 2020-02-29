package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

//@Disabled
@Autonomous(name = "Auto Arm Test", group = "Test")
public class AutoArmTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        robot.autoPivotLeft.setPosition(robot.constants.AUTO_PIVOT_DOWN_POSITION);
        telemetry.addData("Position of pivot is", robot.constants.AUTO_PIVOT_DOWN_POSITION);
        telemetry.update();

        sleep(3000);

        robot.autoPivotLeft.setPosition(robot.constants.AUTO_PIVOT_COMPACT_POSITION);
        telemetry.addData("Position of pivot is", robot.constants.AUTO_PIVOT_COMPACT_POSITION);
        telemetry.update();

        sleep(3000);


        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
        telemetry.addData("Position of claw is", robot.constants.AUTO_COLLECT_OPEN_POSITION);
        telemetry.update();

        sleep(3000);

        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_MID_POSITION);
        telemetry.addData("Position of claw is", robot.constants.AUTO_COLLECT_MID_POSITION);
        telemetry.update();

        sleep(3000);

        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_GRAB_POSITION);
        telemetry.addData("Position of claw is", robot.constants.AUTO_COLLECT_GRAB_POSITION);
        telemetry.update();

        sleep(3000);
    }
}
