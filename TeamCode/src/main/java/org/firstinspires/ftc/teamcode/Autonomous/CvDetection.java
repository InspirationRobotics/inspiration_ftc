package org.firstinspires.ftc.teamcode.Autonomous;
import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "CVDetection", group = "CV")
public class CvDetection extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        robot.setHardwareMap(hardwareMap);
        robot.initHw();
        SkystoneDetector detector = new SkystoneDetector();

        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        while (detector.getDetectionStatus() == 0) {
            setPower(0.20, 0.20);
        }
        setPower(0, 0);
        detector.disable();

    }

}
