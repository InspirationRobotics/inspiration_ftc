package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

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

        long endTime = System.currentTimeMillis() + 4000;

        while((System.currentTimeMillis() < endTime) && opModeIsActive()) {
            telemetry.addData("IMU heading", getHeading());
            telemetry.update();
        }


        wallAlign(1, 20, robot.distanceFrontRight, Direction.FORWARD);

        telemetry.addData("Front distance sensor distance (IN)", robot.distanceFrontRight.getDistance(DistanceUnit.INCH));
        telemetry.update();

        sleep(1000);

        gyroTurn(180, 0.5, 4);


        sleep(1000);

        wallAlign(1, 30, robot.distanceBackLeft, Direction.BACKWARD);


        while(opModeIsActive()) {
            telemetry.addData("Front distance sensor distance (IN)", robot.distanceFrontRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("Back distance sensor distance (IN)", robot.distanceBackLeft.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
