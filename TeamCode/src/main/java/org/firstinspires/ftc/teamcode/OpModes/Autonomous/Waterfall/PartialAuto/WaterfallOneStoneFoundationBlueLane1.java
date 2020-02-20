package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Disabled
@Autonomous(name = "WaterfallOneStoneFoundationBlueLane1", group = "Waterfall")
public class WaterfallOneStoneFoundationBlueLane1 extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        encoderStrafeGyro(20.5, 1, 0);

        gyroTurn(0, 0.5, 1);

        grabBlockBack(); /* empty */ // problem

        encoderStrafeGyro(-4, 1,0);


        // problems the first time, next two lines
        gyroTurn(0, 0.5, 1);

        encoderDrive(75,75,0.6,0.6,6);


        encoderStrafeGyro(4, 1, 0);

        releaseBlockBack();

        encoderStrafeGyro(4, 1,0);

        gyroTurn(90,0.7,2.5); /* error: no gyro turn, but wheels moved slightly */

        encoderDrive(-8,-8,0.7,0.7,2); /* didn't really execute the first time */

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS); /* didn't work */

        sleep(1250);

        // the next three somewhat worked, but grip was lost on the foundation and it ended up failing
        encoderDrive(16,16, 0.4,0.4,3);

        encoderDrive(6,15, 0.4,0.4,3);

        gyroTurn(-180,0.7, 3);

        encoderDrive(-12,-12,0.8,0.8,5);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

        sleep(500);

        encoderDrive(5,5,0.8,0.8,1);

        encoderStrafeGyro(22, 1,-180);

        encoderStrafeGyro(-4,1,-180);

        encoderDrive(40,40,0.8,0.8,5);

    }
}
