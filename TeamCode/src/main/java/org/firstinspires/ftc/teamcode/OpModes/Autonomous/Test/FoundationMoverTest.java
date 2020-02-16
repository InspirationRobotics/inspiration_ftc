package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;


@Disabled
@Autonomous(name = "Foundation Mover Test", group = "Test")
public class FoundationMoverTest extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        gyroTurn(-90, 0.3, 3.5);

        encoderDrive(-5, -5, 0.8,0.8,1.5);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        gyroTurn(-180, 0.3, 5);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        encoderDrive(5, 5, 0.8,0.8,1.5);

        strafeDistSensor(26, Direction.LEFT, robot.distanceRight, 4000);

        encoderDrive(55, 55, 0.6,0.6,1.5);


    }
}

