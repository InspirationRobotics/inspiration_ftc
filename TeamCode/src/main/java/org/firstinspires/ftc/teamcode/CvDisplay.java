package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="CVDisplay", group="CV")
public class CvDisplay extends LinearOpMode {

    public HardwareMap ahwmap;
    public SamplingOrderDetector detector = new SamplingOrderDetector();

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }

    public void runOpMode () {
        setHardwareMap(hardwareMap);
        detector.init(ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
        sleep(4000);
        detector.disable();
    }

}
