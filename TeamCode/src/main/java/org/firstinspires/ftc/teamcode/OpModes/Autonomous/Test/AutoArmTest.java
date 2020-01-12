package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "Auto Arm Test", group = "Test")
public class AutoArmTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        grabBlockBack();

        sleep(1000);

        retractAutoArmBack();

        sleep(3000);

        releaseBlockBack();
    }
}
