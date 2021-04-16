package org.firstinspires.ftc.teamcode.usrtestarea;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.Robot;

/*MECHANICAL PEOPLE (noah): i added two variables: SERVO_POSITION and SERVER_RETRACTED_POSITION
(I don't know how effective the name is lol). Feel free to increase/decrease the variable values as you figure out what
position you need the servos to go to (I just set them arbitrarily). The greater the number the more it turns basically.
the two buttons are gamepad2 a and b lmk if you want me to change it */

@TeleOp(name = "TeleOp 2")
public class TeleopNew extends OpMode {

    NewRobot newrobot = new NewRobot();
    public static final double STRAFE_SPEED = 0.5;
    public static final double SERVO_POSITION = 0.6;
    public static final double SERVO_RETRACTED_POSITION = 0.2;

    @Override
    public void init() {
        newrobot.setHardwareMap(hardwareMap);
        newrobot.initDrivetrain();
        newrobot.initAllServos();
        newrobot.initMiscMotors();
        //robot.wobbleGoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        newrobot.frontLeft.setPower(-gamepad1.left_stick_y);
        newrobot.frontRight.setPower(-gamepad1.right_stick_y);
        newrobot.backLeft.setPower(-gamepad1.left_stick_y);
        newrobot.backRight.setPower(-gamepad1.right_stick_y);

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
            newrobot.shooterOne.setPower(-0.6);
        }
        if (gamepad1.x) {
            newrobot.shooterOne.setPower(0);
        }

//        if(!gamepad1.right_bumper && gamepad1.left_bumper) {
//            newrobot.collector.setPower(-1);
//        } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
//            newrobot.collector.setPower(1);
//        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
//            newrobot.collector.setPower(0);
//        }

//        if (gamepad1.a) {
//            newrobot.shooter.setPosition(SERVO_POSITION);
//        }
//        if (gamepad1.b) {
//            newrobot.shooter.setPosition(SERVO_RETRACTED_POSITION);
//        }
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            //robot.wobbleGoal.setTargetPosition(5);
            //robot.wobbleGoal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            newrobot.wobbleGoal.setPower(0.3);
            /*while (robot.wobbleGoal.isBusy()) {
                telemetry.addData("Im Cool:", true);
                telemetry.update();
            }
            telemetry.addData("Im Cool:", false);
            telemetry.update();
            robot.wobbleGoal.setPower(0);
            robot.wobbleGoal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        } else if(!gamepad2.dpad_up && gamepad2.dpad_down){
            newrobot.wobbleGoal.setPower(-0.5);
        } else if(!gamepad2.dpad_up && !gamepad2.dpad_down){
            newrobot.wobbleGoal.setPower(0);
        }
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            newrobot.servoWobbleGoal.setPosition(0);
        }
        if (!gamepad2.dpad_right && gamepad2.dpad_left) {
            newrobot.servoWobbleGoal.setPosition(1.2);
        }
    }
}