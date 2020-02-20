package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "MoveToSkystoneUnitTest")
public class MoveToSkystone extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        int tgtSS = 2;

        telemetry.addLine("Ready");
        telemetry.addData("Target Stone", tgtSS);
        telemetry.update();

        waitForStart();

        moveToSkystone(tgtSS, AllianceSide.BLUE);
    }
}
