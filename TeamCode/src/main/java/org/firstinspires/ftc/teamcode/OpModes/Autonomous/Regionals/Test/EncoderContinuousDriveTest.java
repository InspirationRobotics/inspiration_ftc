package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "Encoder Continuous Drive Test", group = "Storm")
public class EncoderContinuousDriveTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        AllianceSide allianceSide = AllianceSide.BLUE;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();
        initIMU(hardwareMap);

        initArm();

        telemetry.addLine("Ready To Go!");
        telemetry.update();

        waitForStart();

        setIMUOffset();

        grabAutoArmStorm();
        moveToFoundationStorm(2);

        multipleStoneStorm(5);
        moveToFoundationStorm(5);

        multipleStoneStorm(1);
        moveToFoundationStorm(1);
    }

    //@Disabled
    @Autonomous(name = "IMU Test", group = "Test")
    public static class IMUTest extends BasicExtendedLinearOpMode {

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
}
