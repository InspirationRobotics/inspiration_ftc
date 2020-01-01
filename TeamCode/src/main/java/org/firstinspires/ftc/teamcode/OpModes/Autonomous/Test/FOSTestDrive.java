package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedOpMode;
import org.firstinspires.ftc.teamcode.Hardware.DistanceSensorType;

@Disabled
@Autonomous(name = "FOS Test Drive", group = "FOS")
public class FOSTestDrive extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();
        robot.initDistanceSensors(DistanceSensorType.MODERN_ROBOTICS);
        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        moveToPos(25, 25);
    }
}
