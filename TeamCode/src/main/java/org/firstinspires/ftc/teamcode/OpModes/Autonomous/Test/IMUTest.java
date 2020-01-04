package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

//@Disabled
@Autonomous(name = "IMU Value Test", group = "Test")
public class IMUTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        long endTime = System.currentTimeMillis() + 4000;

        while((System.currentTimeMillis() < endTime) && opModeIsActive()) {
            telemetry.addData("IMU heading", getHeading());
            telemetry.update();
        }

        gyroTurn(180, 0.5, 4);


        sleep(2000);

        gyroTurn(-180, 0.5, 4);

        while(opModeIsActive()) {
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
    }
}
