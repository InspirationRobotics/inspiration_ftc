package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name = "Fall Parking Lane 1 Blue", group = "Waterfall")
public class WaterfallParkingLane1Blue extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();


        //sleep until partner is done with tasks
        //subject to change
        sleep(1500);

        //strafe left until in lane 1
        strafeDistSensor(26, Direction.LEFT, robot.distanceRight, 6000);

        gyroTurn(0, 0.5, 3);
        //move backwards using encoders
        encoderDrive(10, 10, .7, .7, 6);
    }
}