package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Disabled
@Autonomous(name = "Encoder Strafe Test 20in", group = "Test")
public class EncoderStrafeTestDrive20in extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();

        waitForStart();

        encoderStrafe(20, 1);
    }
}
