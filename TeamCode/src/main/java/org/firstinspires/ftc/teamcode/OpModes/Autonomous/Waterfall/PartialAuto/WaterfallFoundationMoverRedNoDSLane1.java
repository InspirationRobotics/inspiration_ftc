package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;

@Autonomous(name = "WaterfallFoundationMoverRedNoDSLane1", group = "Waterfall")
public class WaterfallFoundationMoverRedNoDSLane1 extends ExtendedLinearOpMode {

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

        gyroTurn(104, 0.8, 2);

        encoderDrive(36,36,0.6,0.6,5);
    }
}
