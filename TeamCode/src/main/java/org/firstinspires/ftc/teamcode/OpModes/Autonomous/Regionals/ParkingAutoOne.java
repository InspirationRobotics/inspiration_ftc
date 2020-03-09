package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park Wall 1 Tile", group="Regional")
public class ParkingAutoOne extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {


        AllianceSide allianceSide = AllianceSide.RED;
        alliance = AllianceSide.RED;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        encoderDrive(24, 24, 0.25, 0.25, 7);

        sleep(3000);

    }

}