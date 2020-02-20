package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Autonomous(name = "Basic Full Red Auto", group = "Main")
public class BasicFullRedAuto extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        double endVisionTime = System.currentTimeMillis() + 3000;
        while((System.currentTimeMillis() < endVisionTime) && opModeIsActive() && (detector.skystoneId(AllianceSide.RED) == SkystonePosition.UNKNOWN)) {
            idle();
        }

        SkystonePosition skystonePosition = detector.skystoneId(AllianceSide.RED);
        int skystoneId;

        if (skystonePosition == SkystonePosition.LEFT) {
            skystoneId = 1;
        } else if (skystonePosition == SkystonePosition.CENTER) {
            skystoneId = 2;
        } else if (skystonePosition == SkystonePosition.RIGHT) {
            skystoneId = 3;
        } else {
            skystoneId = 2;
        }

        telemetry.addData("skystone position", skystonePosition);
        telemetry.addData("skystone id", skystoneId);
        telemetry.update();

        moveToSkystone(skystoneId, AllianceSide.RED);

        moveToFoundation(skystoneId, AllianceSide.RED);

//        if (skystoneId == 1) {
//            multipleStoneRevised(skystoneId + 3, AllianceSide.BLUE);
//        }

//        moveFoundation(AllianceSide.RED);
//
//        telemetry.addLine("parking now");
//        telemetry.update();
//
////        sleep(250);
//
//        parkBridge(AllianceSide.RED);

    }

}
