package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

//@Disabled
@Autonomous(name = "IMU Test", group = "Test")
public class IMUTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        setIMUOffset();

        gyroTurn(90,0.3,4);
//        sleep(1000);
//        gyroTurn(180,0.3,4);
//        sleep(1000);
//        gyroTurn(270,0.3,4);
//        sleep(1000);
//        gyroTurn(360,0.3,4);
//        sleep(1000);
//        gyroTurn(0,0.3,4);
//        sleep(1000);
//        gyroTurn(-90,0.3,4);


        while(opModeIsActive()) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("IMU Angle 1", getHeading());
            telemetry.addData("IMU Angle 2", robot.angles.secondAngle);
            telemetry.addData("IMU Angle 3", robot.angles.thirdAngle);
            telemetry.update();
        }
    }
}
