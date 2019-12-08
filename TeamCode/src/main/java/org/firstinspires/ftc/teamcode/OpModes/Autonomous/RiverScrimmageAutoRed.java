package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name="River Scrimmage Auto Red", group = "River")
public class RiverScrimmageAutoRed extends ExtendedLinearOpMode {


    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER_SIMPLE);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        timedDrive(Direction.LEFT, 4000, 1);
        timedDrive(Direction.BACKWARD, 2000, 0.7);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        timedDrive(Direction.RIGHT, 6000, 1);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        timedDrive(Direction.FORWARD, 2000, 0.7);

    }

    void intake() {

    }

    void outtake() {

    }
}
