package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "Encoder Strafe Gyro Test 30in", group = "Test")
public class EncoderStrafeGyroTestDrive30in extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();
        initIMU(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        encoderStrafeGyro(30, 1, 0);
    }
}
