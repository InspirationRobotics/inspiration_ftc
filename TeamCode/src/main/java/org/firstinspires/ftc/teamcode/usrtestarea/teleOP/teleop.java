package org.firstinspires.ftc.teamcode.usrtestarea.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class teleop extends OpMode {

    Robot robot = new Robot();
    public static final double STRAFE_SPEED = 0.5;
    public static final double SERVO_POSITION = 0.6;
    public static final double SERVO_RETRACTED_POSITION = 0.2;

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();
        //robot.wobbleGoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        robot.frontLeft.setPower(-gamepad1.left_stick_y);
        robot.frontRight.setPower(-gamepad1.right_stick_y);
        robot.backLeft.setPower(-gamepad1.left_stick_y);
        robot.backRight.setPower(-gamepad1.right_stick_y);

        if (gamepad1.left_trigger >= 0.5 && gamepad1.right_trigger < 0.5) {
            robot.frontLeft.setPower(-STRAFE_SPEED);
            robot.frontRight.setPower(STRAFE_SPEED);
            robot.backLeft.setPower(STRAFE_SPEED);
            robot.backRight.setPower(-STRAFE_SPEED);
        } else if (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >= 0.5) {
            robot.frontLeft.setPower(STRAFE_SPEED);
            robot.frontRight.setPower(-STRAFE_SPEED);
            robot.backLeft.setPower(-STRAFE_SPEED);
            robot.backRight.setPower(STRAFE_SPEED);
        } else {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

        if (gamepad1.y) {
            robot.shooterOne.setPower(-0.6);
        }
        if (gamepad1.x) {
            robot.shooterOne.setPower(0);
        }

        if(!gamepad1.right_bumper && gamepad1.left_bumper) {
            robot.collector.setPower(-1);
        } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
            robot.collector.setPower(1);
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.collector.setPower(0);
        }

        if (gamepad1.a) {
            robot.shooter.setPosition(SERVO_POSITION);
        }
        if (gamepad1.b) {
            robot.shooter.setPosition(SERVO_RETRACTED_POSITION);
        }
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            //robot.wobbleGoal.setTargetPosition(5);
            //robot.wobbleGoal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleGoal.setPower(0.3);
            /*while (robot.wobbleGoal.isBusy()) {
                telemetry.addData("Im Cool:", true);
                telemetry.update();
            }
            telemetry.addData("Im Cool:", false);
            telemetry.update();
            robot.wobbleGoal.setPower(0);
            robot.wobbleGoal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        } else if(!gamepad1.dpad_up && gamepad1.dpad_down){
            robot.wobbleGoal.setPower(-0.5);
        } else if(!gamepad1.dpad_up && !gamepad1.dpad_down){
            robot.wobbleGoal.setPower(0);
        }
        if (gamepad1.dpad_right && !gamepad1.dpad_left) {
            robot.servoWobbleGoal.setPosition(0);
        }
        if (!gamepad1.dpad_right && gamepad1.dpad_left) {
            robot.servoWobbleGoal.setPosition(1.2);
        }

    }
}