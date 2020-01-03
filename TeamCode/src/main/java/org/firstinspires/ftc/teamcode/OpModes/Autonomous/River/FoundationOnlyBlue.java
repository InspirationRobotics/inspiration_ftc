package org.firstinspires.ftc.teamcode.OpModes.Autonomous.River;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Foundation Mover Blue", group = "River")
public class FoundationOnlyBlue extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);
        initDetector();
        //robot.initIMU();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        while(robot.distanceFront.getDistance(DistanceUnit.INCH) > 90 && opModeIsActive())
        {
            setPower(0.1, 0.1);
        }

        //turn left (counter-clockwise)
        doEncoderTurn(0.5, 180);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        while(robot.distanceFront.getDistance(DistanceUnit.INCH) > 6 && opModeIsActive())
        {
            setPower(0.1, 0.1);
        }
        setPower(0, 0);

        //turn left (counter-clockwise)
        gyroTurn(-90,.1, 6);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        while(robot.distanceLeft.getDistance(DistanceUnit.INCH) > 6 && opModeIsActive())
        {
            strafeGyro(90, robot.getHeading());
        }

        strafeGyro(0, robot.getHeading());

        while(robot.distanceFront.getDistance(DistanceUnit.INCH) > 70 && opModeIsActive())
        {
            setPower(0.1, 0.1);
        }
        setPower(0, 0);

    }
}

