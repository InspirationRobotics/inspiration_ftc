package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Disabled
@Autonomous(name = "WaterfallFoundationMoverRedNoDSLane2", group = "Waterfall")
public class WaterfallFoundationMoverRedNoDSLane2 extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        encoderDrive(-30,-30,0.7,0.7,4);

        encoderDrive(-5,-5,0.2,0.2,2);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

        sleep(1000);

        encoderDrive(16,16, 0.4,0.4,3);

        encoderDrive(15,6, 0.4,0.4,3);

        gyroTurn(90,0.7, 3);

        encoderDrive(-24,-24,0.8,0.8,5);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

        sleep(500);

        encoderDrive(5,5,0.6,0.6,2);

        long endTime = System.currentTimeMillis() + 1500;

        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            strafeNoAngle(1);
        }

        encoderStrafeGyro(4, 1, 90);

        gyroTurn(90, 0.7, 2);

        encoderDrive(42,42,0.6,0.6,5);
    }
}
