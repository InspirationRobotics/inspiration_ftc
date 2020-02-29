package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "Foundation And Park Red", group = "Storm")
public class FoundationAndParkRed extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        AllianceSide allianceSide = AllianceSide.RED;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();
        initIMU(hardwareMap);

        telemetry.addLine("Ready To Go!");
        telemetry.update();

        waitForStart();

        setIMUOffset();

        encoderDriveBasicGyro(16,1,0,4);

        encoderDrive(-8,-8,0.5,0.5,2);

        encoderDrive(30,30,0.4,0.4,5);

        encoderDrive(10, 10, 0.3, 0.3, 10);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

        sleep(1000);

//            encoderDrive(-5, -5, 0.7, 0.7, 10);

        encoderDrive(-26, -26, 0.8, 0.8, 10);

        gyroTurn(-90, 0.8, 10);

        robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            sleep(250);

        encoderDrive(-5, -5, 1, 1, 0.8);


        gyroTurn(0,0.4,3);

        encoderDrive(-10,-10,0.7,0.7,2);

        encoderDrive(22,22,0.5,0.5,5);

        gyroTurn(-90,0.2,3);

        encoderDrive(-25,-25,0.6,0.6,5);
    }
}
