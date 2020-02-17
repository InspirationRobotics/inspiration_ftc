package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Strafe Angle Test", group = "Test")
public class TestStrafe extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);


        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

//        double lfSpeed;
//        double lbSpeed;
//        double rfSpeed;
//        double rbSpeed;
//
//        double startHeading = robot.getHeading();
//        double currentHeading;
//
//        while (opModeIsActive()) {
//            lfSpeed = 0.9;
//            lbSpeed = -0.9;
//            rfSpeed = -0.9;
//            rbSpeed = 0.9;
//
//            currentHeading = robot.getHeading();
//            if(Math.abs(startHeading-currentHeading) > 1) {
//
//            }
//        }
//
//        stopMotors();

        double endTime = System.currentTimeMillis() + 5000;

        while((System.currentTimeMillis() < endTime) && opModeIsActive()) {
            strafeAngleGyro(320, 0);
        }
    }
}
