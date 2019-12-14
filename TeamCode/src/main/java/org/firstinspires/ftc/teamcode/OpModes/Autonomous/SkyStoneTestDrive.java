package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.inspiration.inspcv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name="SkystoneTestDrive", group="Competition")
public class SkyStoneTestDrive extends ExtendedLinearOpMode {
    public SkystoneDetector detector = new SkystoneDetector();

    @Override
    public void runOpMode () {
        robot.setHardwareMap(hardwareMap);
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        robot.initDrivebase();
        waitForStart();

        while(!detector.isVerifiedSkystone("blue")){setPower(-0.50, -0.50);}
        setPower(0, 0);
        detector.disable();
    }
}
