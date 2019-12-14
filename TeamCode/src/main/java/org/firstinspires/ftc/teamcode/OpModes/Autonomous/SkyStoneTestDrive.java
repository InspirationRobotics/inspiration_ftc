package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.inspiration.inspcv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name="SkystoneTestDrive", group="Competition")
public class SkyStoneTestDrive extends ExtendedLinearOpMode {
    public SkystoneDetector detector = new SkystoneDetector();
    int initialPos;

    @Override
    public void runOpMode () {
        robot.setHardwareMap(hardwareMap);
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        robot.initDrivebase();
        waitForStart();
        initialPos = robot.leftBack.getCurrentPosition();

        while(!detector.isVerifiedSkystone("blue")){
            setPower(-0.20, -0.20);
            telemetry.addData("encoders", robot.leftBack.getCurrentPosition() - initialPos);
            telemetry.update();
        }
        setPower(0, 0);
        detector.disable();
        sleep(2000);

    }
}