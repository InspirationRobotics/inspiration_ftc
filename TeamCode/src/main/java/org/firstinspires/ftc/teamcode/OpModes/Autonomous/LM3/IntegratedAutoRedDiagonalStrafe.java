package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Disabled
@Autonomous(name = "League Championships Red")
public class IntegratedAutoRedDiagonalStrafe extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

//        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);

        inspectFrontDistanceSensors(28);
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
            skystoneId = 3;
        } else if (skystonePosition == SkystonePosition.CENTER) {
            skystoneId = 2;
        } else if (skystonePosition == SkystonePosition.RIGHT) {
            skystoneId = 1;
        } else {
            skystoneId = 2;
        }

        telemetry.addData("skystone position", skystonePosition);
        telemetry.addData("skystone id", skystoneId);
        telemetry.update();

        moveToSkystoneRevised(skystoneId, AllianceSide.RED);

        moveToFoundationDiagonalStrafe(skystoneId, AllianceSide.RED);

//        if (skystoneId == 1) {
//            multipleStoneRevised(skystoneId + 3, AllianceSide.RED);
//            moveToFoundationDiagonalStrafe(skystoneId + 3, AllianceSide.RED);
//        }

        moveFoundation(AllianceSide.RED);

        telemetry.addLine("parking now");
        telemetry.update();

//        sleep(250);

        parkBridgeRevised(AllianceSide.RED);

    }
}