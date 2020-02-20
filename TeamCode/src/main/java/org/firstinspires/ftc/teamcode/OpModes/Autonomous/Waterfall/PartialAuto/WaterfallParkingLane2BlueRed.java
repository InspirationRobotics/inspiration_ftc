package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Fall Parking Lane 2", group = "Waterfall")
public class WaterfallParkingLane2BlueRed extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        //sleep until partner is done with tasks
        //subject to change
        sleep(1500);

        //move backwards using encoders
        encoderDrive(3, 3, .7, .7, 6);
    }
}