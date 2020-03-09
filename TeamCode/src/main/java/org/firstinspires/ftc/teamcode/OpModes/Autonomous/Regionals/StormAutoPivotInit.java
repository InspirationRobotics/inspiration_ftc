package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "Storm Auto Pivot Init", group = "Storm")
public class StormAutoPivotInit extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        AllianceSide allianceSide = AllianceSide.BLUE;
        alliance = AllianceSide.BLUE;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();

        waitForStart();

        robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_DOWN_POSITION);
        robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_DOWN_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("Left Auto Pivot Pos (Down)", robot.autoPivotLeft.getPosition());
            telemetry.addData("Right Auto Pivot Pos (Down)", robot.autoPivotRight.getPosition());
            telemetry.update();
        }
    }
}
