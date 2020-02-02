package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "Encoder Drive Test", group = "Test")
public class EncoderDriveTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();

        waitForStart();

        encoderDrive(20, 20, 0.4, 0.4, 5);

        sleep(5000);

        encoderDrive(-15, -15, 0.4, 0.4, 5);

    }
}
