package org.firstinspires.ftc.teamcode.usrtestarea.pahel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

/*MECHANICAL PEOPLE (noah): i added two variables: SERVO_POSITION and SERVER_RETRACTED_POSITION
(I don't know how effective the name is lol). Feel free to increase/decrease the variable values as you figure out what
position you need the servos to go to (I just set them arbitrarily). The greater the number the more it turns basically.
the two buttons are gamepad2 a and b lmk if you want me to change it */

@TeleOp(name = "TeleOp wb")
public class teleop extends CommonAutoFunctions {

    Robot robot = new Robot();
    public static final double STRAFE_SPEED = 0.5;
    public static final double SERVO_POSITION = 0.6;
    public static final double SERVO_RETRACTED_POSITION = 0.2;

    @Override
    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();

        waitForStart();

        while (opModeIsActive()) {

            robot.frontLeft.setPower(-gamepad1.left_stick_y);
            robot.frontRight.setPower(-gamepad1.right_stick_y);
            robot.backLeft.setPower(-gamepad1.left_stick_y);
            robot.backRight.setPower(-gamepad1.right_stick_y);

            while (gamepad1.dpad_left) {
                robot.frontLeft.setPower(-STRAFE_SPEED);
                robot.frontRight.setPower(STRAFE_SPEED);
                robot.backLeft.setPower(STRAFE_SPEED);
                robot.backRight.setPower(-STRAFE_SPEED);

                if (!gamepad1.dpad_left) {
                    break;
                }
            }
            while (gamepad1.dpad_right) {
                robot.frontLeft.setPower(STRAFE_SPEED);
                robot.frontRight.setPower(-STRAFE_SPEED);
                robot.backLeft.setPower(-STRAFE_SPEED);
                robot.backRight.setPower(STRAFE_SPEED);

                if (!gamepad1.dpad_right) {
                    break;
                }
            }

            if (gamepad1.y) {
                robot.shooterOne.setPower(-1);
            }
            if (gamepad1.x) {
                robot.shooterOne.setPower(0);
            }

            while (gamepad1.right_bumper) {
                robot.collector.setPower(-1);
                if (!gamepad1.right_bumper) {
                    robot.collector.setPower(0);
                    break;
                }
            }
            while (gamepad1.left_bumper) {
                robot.collector.setPower(1);
                if (!gamepad1.right_bumper) {
                    robot.collector.setPower(0);
                    break;
                }
            }

            if (gamepad1.a) {
                robot.shooter.setPosition(SERVO_POSITION);
            }
            if (gamepad1.b) {
                robot.shooter.setPosition(SERVO_RETRACTED_POSITION);
            }

        }

        idle();
    }
}