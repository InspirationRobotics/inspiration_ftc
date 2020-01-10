package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.inspiration.inspcv.CameraViewDisplay;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="CVDisplay", group="CV")
//@Disabled
public class SkyStoneVisionTest extends ExtendedLinearOpMode {

    public HardwareMap ahwmap;
    public SkystoneDetector detector = new SkystoneDetector();

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }

    public void runOpMode () {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        setHardwareMap(hardwareMap);
        detector.init(ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();



        while(!detector.isVerifiedSkystone("red")) {
            setPower(0.2, 0.2);
        }

        stopMotors();

        while (opModeIsActive()) {
            telemetry.addData("detection:", detector.isVerifiedSkystone("red"));
            telemetry.addData("front distance sensor", robot.distanceFrontRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("back distance sensor", robot.distanceBackLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("coordinates black", detector.returnCoords()[0]);
            telemetry.addData("coordinates gold", detector.returnCoords()[1]);
            telemetry.addData("id:", detector.skystoneId());
            telemetry.update();
            sleep(20);
            if (isStopRequested()) {
                detector.disable();
            }
        }

    }

}
