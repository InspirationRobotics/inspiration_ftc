package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

//@Disabled
@Autonomous(name = "Encoder Strafe Test 20in", group = "Test")
public class EncoderStrafeTestDrive20in extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        initIMU(hardwareMap);

        waitForStart();

        encoderStrafeGyro(20, 1,0,4);
    }
}
