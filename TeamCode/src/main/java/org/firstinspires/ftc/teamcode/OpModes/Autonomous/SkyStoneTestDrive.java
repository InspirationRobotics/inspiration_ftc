package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.inspiration.inspcv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.DistanceSensorType;

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
        robot.initDistanceSensors(DistanceSensorType.REV);
        waitForStart();
        initialPos = robot.leftBack.getCurrentPosition();

        while(!detector.isVerifiedSkystone("blue")){
            strafeNoAngle(-0.8);
            telemetry.addData("encoders", robot.leftBack.getCurrentPosition() - initialPos);
            telemetry.update();
        }
        strafeNoAngle(0);
        detector.disable();
        telemetry.addData("id:", getSkystoneId());
        sleep(10000);

    }
}