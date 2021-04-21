package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewRobot;

@TeleOp
public class ShooterVelocityTuner extends OpMode {

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

        if (gamepad1.y) {
            shooterSpeed = shooterSpeed - 0.1;
        }
        if (gamepad1.x) {
            shooterSpeed = shooterSpeed + 0.1;
        }

        newrobot.shooterOne.setVelocity(shooterSpeed, AngleUnit.DEGREES);
        newrobot.shooterTwo.setVelocity(shooterSpeed, AngleUnit.DEGREES);

        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            newrobot.shooterTilt.setPosition(0);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            newrobot.shooterTilt.setPosition(1);
        }
        if (gamepad1.a && !gamepad1.b) {
            newrobot.magazine.setPosition(0.5);
        } else if (gamepad1.b && !gamepad1.a) {
            newrobot.magazine.setPosition(1);
        }

    }
}