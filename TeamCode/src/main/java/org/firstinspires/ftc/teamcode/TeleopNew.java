package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewRobot;

@TeleOp(name = "TeleOp the Sequel")
public class TeleopNew extends OpMode {

    NewRobot newrobot = new NewRobot();
    public static final double STRAFE_SPEED = 0.5;
    public static final double tiltPos = 1;
    public static final double tiltRetractPos = 0;
    public static final double magPos = 0.6;
    public static final double magRetractPos = 0.2;
    public double shooterSpeed = 0;
    public boolean tiltToggle = false;
    public boolean magToggle = false;


    @Override
    public void init() {
        newrobot.setHardwareMap(hardwareMap);
        newrobot.initDrivetrain();
        newrobot.initAllServos();
        newrobot.initMiscMotors();
    }

    @Override
    public void loop() {
        newrobot.frontLeft.setPower(-gamepad1.left_stick_y);
        newrobot.frontRight.setPower(-gamepad1.right_stick_y);
        newrobot.backLeft.setPower(-gamepad1.left_stick_y);
        newrobot.backRight.setPower(-gamepad1.right_stick_y);
        newrobot.wobbleGoal.setPower(gamepad2.right_stick_y * 0.7);


        if (gamepad1.left_trigger >= 0.5 && gamepad1.right_trigger < 0.5) {
            newrobot.frontLeft.setPower(-STRAFE_SPEED);
            newrobot.frontRight.setPower(STRAFE_SPEED);
            newrobot.backLeft.setPower(STRAFE_SPEED);
            newrobot.backRight.setPower(-STRAFE_SPEED);
        } else if (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >= 0.5) {
            newrobot.frontLeft.setPower(STRAFE_SPEED);
            newrobot.frontRight.setPower(-STRAFE_SPEED);
            newrobot.backLeft.setPower(-STRAFE_SPEED);
            newrobot.backRight.setPower(STRAFE_SPEED);
        } else {
            newrobot.frontLeft.setPower(0);
            newrobot.frontRight.setPower(0);
            newrobot.backLeft.setPower(0);
            newrobot.backRight.setPower(0);
        }

        if (gamepad1.y) {
            shooterSpeed = -1;
            newrobot.shooterOne.setPower(shooterSpeed);
            newrobot.shooterTwo.setPower(shooterSpeed);
        }
        if (gamepad1.x) {
            shooterSpeed = 0;
            newrobot.shooterOne.setPower(shooterSpeed);
            newrobot.shooterTwo.setPower(shooterSpeed);
        }
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            newrobot.shooterTilt.setPosition(0);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            newrobot.shooterTilt.setPosition(1);
        }
        if (gamepad2.a && !gamepad2.b) {
            newrobot.magazine.setPosition(0.5);
        } else if (gamepad2.b && !gamepad2.a) {
            newrobot.magazine.setPosition(1);
        }
        if(!gamepad1.right_bumper && gamepad1.left_bumper) {
            newrobot.collector.setPower(-1);
        } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
            newrobot.collector.setPower(1);
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
           newrobot.collector.setPower(0);
       }
        if(!gamepad2.right_bumper && gamepad2.left_bumper) {
            newrobot.servoWobbleGoal.setPosition(1.2);
        }
        if (!gamepad2.left_bumper && gamepad2.right_bumper) {
            newrobot.servoWobbleGoal.setPosition(0.2);
        }
    }
}