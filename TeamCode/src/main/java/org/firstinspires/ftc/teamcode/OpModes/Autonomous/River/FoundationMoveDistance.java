package org.firstinspires.ftc.teamcode.OpModes.Autonomous.River;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Foundation Mover", group = "River")
public class FoundationMoveDistance extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);
        initDetector();
        //robot.initIMU();
        //initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        while(robot.distanceLeft.getDistance(DistanceUnit.INCH) > 16 && opModeIsActive())
        {
            //strafeGyro(90, robot.getHeading());
            strafe(180);
        }

        stopMotors();

        //turn right (clockwise, 90)
        //doEncoderTurn(0.2, -90);
        //gyroTurn(180, .1, 6);

        while(robot.distanceBack.getDistance(DistanceUnit.INCH) < 50)
        {
            setPower(0.1, 0.1);
        }
        setPower(0, 0);

        extend(robot.constants.EXTENSION_EXTEND_SPEED, 3);

        intake(robot.constants.INTAKE_MOTOR_OUTTAKE_SPEED, 1, false);

        extend(robot.constants.EXTENSION_COMPACT_SPEED, 3);

        //turn left (counter-clockwise)
        //gyroTurn(180, .1, 6);
        doEncoderTurn(0.5, 180);

        encoderDrive(-6,-6,0.7,0.7,3);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        while(robot.distanceRight.getDistance(DistanceUnit.INCH) < 43)
        {
            strafe(180);
        }
        setPower(0, 0);

        //turn left (counter-clockwise)
        //gyroTurn(-90,.1, 6);
        doEncoderTurn(0.4, -90);

        while(robot.distanceRight.getDistance(DistanceUnit.INCH) > 18)
        {
            //strafeGyro(90, robot.getHeading());
            strafe(90);
        }
        strafeGyro(0, 0);

        while(robot.distanceBack.getDistance(DistanceUnit.INCH) > 18)
        {
            //strafeGyro(90, robot.getHeading());
            strafe(270);
        }
        strafe(0);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);


        while(robot.distanceRight.getDistance(DistanceUnit.INCH) > 30)
        {
            setPower(0.1, 0.1);
        }
        setPower(0, 0);

    }
}

