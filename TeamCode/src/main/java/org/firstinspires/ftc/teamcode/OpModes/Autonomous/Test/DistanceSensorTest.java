package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Disabled
@Autonomous(name = "DistanceSensorTest", group = "Test")
public class DistanceSensorTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            readDS(robot.distanceBackRight);
        }

    }
}
