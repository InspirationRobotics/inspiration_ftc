package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Wall Align Test", group = "Test")
public class WallAlignTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode(){

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        wallAlign(0.8, 18, robot.distanceFrontLeft, Direction.FORWARD, 5000);

        while(opModeIsActive()) {
            telemetry.addData("Front distance sensor distance (IN)", robot.distanceFrontRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("Back distance sensor distance (IN)", robot.distanceBackLeft.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
