package org.firstinspires.ftc.teamcode.OpModes.Autonomous.River;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "River Auto Blue Full", group = "River")
public class RiverFullAutoBlue extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);
        initDetector();
        //robot.initIMU();

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        intake(robot.constants.INTAKE_MOTOR_OUTTAKE_SPEED, 1, false);

        encoderDrive(22, 22, 0.8, 4, 6);

        while (!skyStoneIsVisible(AllianceSide.BLUE)) {
            strafeGyro(0, 0);
        }

        stopMotors();

        robot.intake.setPower(robot.constants.INTAKE_MOTOR_INTAKE_SPEED);

        extend(robot.constants.EXTENSION_EXTEND_SPEED, 3);

        intake(robot.constants.INTAKE_MOTOR_INTAKE_SPEED, 4, true);

        extend(robot.constants.EXTENSION_COMPACT_SPEED, 3);

        robot.grabber.setPosition(robot.constants.GRABBER_GRAB_POS);

        gyroTurn(90, 0.5, 4);

        //wallAlign(22, Direction.BACKWARD, robot.distanceBack, 90, 10);

        gyroTurn(180, 0.5, 4);

        encoderDrive(-16, -16, 0.8, 4, 6);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        lift(robot.constants.LIFT_INCREASE_HEIGHT_SPEED, 3);

        robot.wrist.setPosition(robot.constants.WRIST_OUTSIDE_POS);

        robot.grabber.setPosition(robot.constants.GRABBER_OPEN_POS);

        robot.wrist.setPosition(robot.constants.WRIST_INSIDE_POS);

        lift(robot.constants.LIFT_LOWER_HEIGHT_SPEED, 3);

        encoderDrive(30, 30, 0.8, 5, 6);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        encoderDrive(2, 2, 0.8, 1, 6);

        //wallAlign(36, Direction.LEFT, robot.distanceRight, 180, 7);

        encoderDrive(-24, -24, 0.7, 4, 6);

        //wallAlign(58, Direction.LEFT, robot.distanceRight, 180, 6);

        stopMotors();
    }
}
