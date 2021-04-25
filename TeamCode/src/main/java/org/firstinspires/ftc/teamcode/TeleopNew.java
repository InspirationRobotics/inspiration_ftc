package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewRobot;

@TeleOp(name = "TeleOp the Sequel")
public class TeleopNew extends OpMode {

    NewRobot newrobot = new NewRobot();
    public static final double STRAFE_SPEED = 0.7;
    public double shooterSpeed = 0;
    public boolean tiltToggle = false;

    Servo whisker;

    @Override
    public void init() {
        newrobot.setHardwareMap(hardwareMap);
        newrobot.initDrivetrain();
        newrobot.initAllServos();
        newrobot.initMiscMotors();
        whisker = hardwareMap.servo.get("whisker");
    }

    @Override
    public void start() {
        whisker.setPosition(1);
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
            newrobot.shooterOne.setVelocity(-196, AngleUnit.DEGREES);
            newrobot.shooterTwo.setVelocity(-196, AngleUnit.DEGREES);
        }
        if (gamepad1.x) {
            newrobot.shooterOne.setVelocity(0, AngleUnit.DEGREES);
            newrobot.shooterTwo.setVelocity(0, AngleUnit.DEGREES);
        }
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            newrobot.shooterTilt.setPosition(0.4);
            tiltToggle = true;
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            newrobot.shooterTilt.setPosition(1);
            tiltToggle = false;
        }
        if (gamepad2.a && !gamepad2.b) {
            if(tiltToggle){
                newrobot.magazine.setPosition(0.5);
            }
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