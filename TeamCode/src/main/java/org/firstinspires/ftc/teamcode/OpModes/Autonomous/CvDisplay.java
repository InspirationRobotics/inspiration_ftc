package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.inspiration.inspcv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="CVDisplay", group="CV")
public class CvDisplay extends LinearOpMode {

    public HardwareMap ahwmap;
    public SkystoneDetector detector = new SkystoneDetector();

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }

    public void runOpMode () {
        waitForStart();

        setHardwareMap(hardwareMap);
        detector.init(ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        while (opModeIsActive()) {
            telemetry.addData("detection:", detector.isVerifiedSkystone());
            telemetry.addData("coordinates black", detector.returnCoords()[0]);
            telemetry.addData("coordinates gold", detector.returnCoords()[1]);
            telemetry.update();
            sleep(20);
            if (isStopRequested()) {
                detector.disable();
            }
        }

    }

}
