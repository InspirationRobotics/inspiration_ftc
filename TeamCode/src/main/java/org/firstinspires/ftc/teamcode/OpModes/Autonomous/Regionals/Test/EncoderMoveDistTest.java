package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;

@Autonomous(name = "Encoder Move Dist Test", group = "Test")
public class EncoderMoveDistTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        initIMU(hardwareMap);
        setIMUOffset();

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        int i = 0;
        while (i < 2) {
            EncoderMoveDist(1, 24, true, true, 0);
            EncoderMoveDist(1, -24, true, true, 0);
            i++;
        }



    }
}
