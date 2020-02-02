package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "Encoder Strafe Test 10in", group = "Test")
public class EncoderStrafeTestDrive10in extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();

        waitForStart();

        encoderStrafe(10, 1);
    }
}
