package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "WaterfallOneStoneFoundationBlueLane2", group = "Waterfall")
public class WaterfallOneStoneFoundationBlueLane2 extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        encoderStrafe(20.5, 0.75);

        gyroTurn(0, 0.5, 1);

        grabBlockBack(); /* empty */

        encoderStrafe(-4, 0.75);

        encoderDrive(-75,-75,0.6,0.6,6);

        encoderStrafe(4, 0.75);

        releaseBlockBack();

        encoderStrafe(-4, 0.75);

        gyroTurn(-90,0.7,2.5);

        encoderDrive(-8,-8,0.7,0.7,2);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

        sleep(1250);

        encoderDrive(16,16, 0.4,0.4,3);

        encoderDrive(6,15, 0.4,0.4,3);

        gyroTurn(-180,0.7, 3);

        encoderDrive(-12,-12,0.8,0.8,5);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

        sleep(500);

        encoderDrive(5,5,0.8,0.8,1);

        encoderStrafe(22, 0.8);

        encoderDrive(40,40,0.8,0.8,5);

    }
}
