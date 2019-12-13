package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

public class IMUTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER_SIMPLE);
        robot.initIMU();

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        double startAngle = robot.getHeading();
        double targetAngle = startAngle + 90;

        gyroTurn(targetAngle, 0.65, 4);
    }
}
