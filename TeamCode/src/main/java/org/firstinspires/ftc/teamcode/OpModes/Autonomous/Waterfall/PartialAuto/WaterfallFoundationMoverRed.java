package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name = "Fall Foundation Mover Red", group = "Waterfall")
public class WaterfallFoundationMoverRed extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        //sleep(750);

        //moving backwards to align with foundation
        wallAlign(0.3, 30, robot.distanceFrontLeft, Direction.BACKWARD, 6000);

        encoderDrive(-4,-4,0.3,0.3,2);

        gyroTurn(5, 0.3, 1);

        gyroTurn(2, 0.5, 1);

        encoderDrive(-2,-2,0.3,0.3,2);

        //grab foundation
        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

        sleep(3000);

        wallAlign(0.9, 10, robot.distanceFrontLeft, Direction.FORWARD, 3000);

        //turning clockwise 90 degrees to move foundation into
        gyroTurn(90, 1, 7);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

        sleep(750);

        encoderDrive(5,5,0.7,0.7,2);

        gyroTurn(90, 0.7, 2.5);

        //move forward to park
        strafeDistSensor(27, Direction.RIGHT, robot.distanceLeft, 5000);

        gyroTurn(92,0.7,2);

        encoderDrive(40,40,0.7,0.7,6);
    }
}

