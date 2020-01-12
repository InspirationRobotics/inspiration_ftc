package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "Encoder Strafe Test 20in", group = "Test")
public class EncoderStrafeTestDrive20in extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.initDrivebase();

        waitForStart();

        encoderStrafe(20, 0.75);
    }
}
