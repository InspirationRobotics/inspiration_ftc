package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park Wall 0.5 Tile", group="Regional")
public class ParkingAutoPointFive extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {


        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        encoderDrive(12, 12, 0.25, 0.25, 7);

        sleep(3000);

    }

}