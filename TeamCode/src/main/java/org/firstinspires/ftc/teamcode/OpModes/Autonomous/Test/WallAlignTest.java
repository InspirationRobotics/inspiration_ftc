package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name = "Wall Align Test", group = "Test")
public class WallAlignTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode(){

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();
    }
}
