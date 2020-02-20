package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "AAAAAAAAAAAAAAAA", group = "Basic")
public class MoveToSkystone extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        moveToSkystone(3, AllianceSide.RED);
    }
}
