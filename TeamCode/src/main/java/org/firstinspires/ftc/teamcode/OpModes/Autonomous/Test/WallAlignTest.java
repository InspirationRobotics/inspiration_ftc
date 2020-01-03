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
        robot.initRiver(RobotVersion.RIVER);
        initIMU(hardwareMap);


        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        wallAlign(1, 20, robot.distanceFront, Direction.FORWARD);

        telemetry.addData("Front distance sensor distance (IN)", robot.distanceFront.getDistance(DistanceUnit.INCH));
        telemetry.update();

        sleep(1000);

        gyroTurn(180, 0.6, 8);

        sleep(1000);

        wallAlign(1, 30, robot.distanceBack, Direction.BACKWARD);


        while(opModeIsActive()) {
            telemetry.addData("Front distance sensor distance (IN)", robot.distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Back distance sensor distance (IN)", robot.distanceBack.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
