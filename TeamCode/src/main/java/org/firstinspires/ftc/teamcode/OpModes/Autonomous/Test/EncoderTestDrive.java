package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Disabled
@Autonomous(name="Encoder Test Drive", group="Test")
public class EncoderTestDrive extends ExtendedLinearOpMode {

    @Override
    public void runOpMode () {
        robot.setHardwareMap(hardwareMap);
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        robot.initDrivebase();
        waitForStart();

        while(!detector.isVerifiedSkystone(AllianceSide.BLUE)){setPower(0.20, 0.20);}
        setPower(0, 0);
        detector.disable();

        encoderDrive(8, 8, 0.1, .1, 6);

        stopMotors();

    }
}