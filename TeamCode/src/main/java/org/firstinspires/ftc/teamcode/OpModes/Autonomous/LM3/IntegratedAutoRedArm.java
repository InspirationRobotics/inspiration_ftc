package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Autonomous(name = "IntegratedAutoRedArm")
public class IntegratedAutoRedArm extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

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

        moveToSkystoneCopy(skystoneId, 1, AllianceSide.RED);

//grabbing stones
        sleep(100);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);

        sleep(1400);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);

        sleep(750);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);

        robot.frontPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        moveToFoundation(skystoneId,1, AllianceSide.RED);

//deposit stones

        sleep(100);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);

        sleep(1000);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_OPEN);

        sleep(750);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);


        moveFoundation(AllianceSide.RED);

        parkBridge(AllianceSide.RED);

    }
}
