package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;

@Autonomous(name = "Encoder Unit Test", group = "Test")
public class EncoderUnitTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        encoderDrive(20,20,0.5,0.5,4);

        sleep(1500);

        encoderDrive(-20,-20,0.5,0.5,4);


        sleep(1500);


        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(1000);

        while(opModeIsActive()) {
            setPower(0.1,0.1);

            telemetry.addData("Right Front Pos", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left Front Pos", robot.rightBack.getCurrentPosition());
            telemetry.addData("Right Back Pos", robot.leftFront.getCurrentPosition());
            telemetry.addData("Left Back Pos", robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
    }
}
