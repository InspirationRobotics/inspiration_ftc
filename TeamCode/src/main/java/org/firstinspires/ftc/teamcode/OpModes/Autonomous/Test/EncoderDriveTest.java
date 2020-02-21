package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

//@Disabled
@Autonomous(name = "Encoder Drive Test", group = "Test")
public class EncoderDriveTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();

        waitForStart();

        encoderDrive(20, 20, 0.8, 0.8, 5);

        sleep(5000);

        encoderDrive(-15, -15, 0.4, 0.4, 5);

    }
}
