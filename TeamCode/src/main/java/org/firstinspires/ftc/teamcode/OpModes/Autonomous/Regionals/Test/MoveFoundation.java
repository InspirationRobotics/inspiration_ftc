package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "MoveFoundationUnitTest")
public class MoveFoundation extends BasicExtendedLinearOpMode {

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

        moveFoundation(AllianceSide.BLUE);

    }

}
