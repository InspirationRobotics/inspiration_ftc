package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.inspiration.inspcv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="CV Test Run", group="CV")
public class SkyStoneTestDrive extends ExtendedLinearOpMode {

    public SkystoneDetector detector = new SkystoneDetector();

    @Override
    public void runOpMode () {
        robot.setHardwareMap(hardwareMap);
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        robot.initDrivebase();
        waitForStart();

        while(!detector.isVerifiedSkystone()){setPower(0.20, 0.20);}
        setPower(0, 0);
        detector.disable();

    }
}
