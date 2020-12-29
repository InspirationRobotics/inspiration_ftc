package org.firstinspires.ftc.teamcode.usrtestarea.pahel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp wb")
public class teleop extends LinearOpMode {

    //drivetrain
    public DcMotor frontLeft;        /* p0 */
    public DcMotor frontRight;       /* p1 */
    public DcMotor backLeft;         /* p2 */
    public DcMotor backRight;

    //servos
    public Servo shooter;            /* p0 */

    public static final double STRAFE_SPEED = 0.5;


    @Override
    public void runOpMode(){

        //drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //servos
        //need to figure out how the servo moves and stuff, will probably go to the lab on 12/30/2020
        //shooter = hardwareMap.get(Servo.class, "shooter");

        waitForStart();

        while(opModeIsActive()){

            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.right_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);

            while (gamepad1.dpad_left){
                frontLeft.setPower(STRAFE_SPEED);
                frontRight.setPower(-STRAFE_SPEED);
                backLeft.setPower(-STRAFE_SPEED);
                backRight.setPower(STRAFE_SPEED);

                if(!gamepad1.dpad_left){
                    break;
                }
            }
            while(gamepad1.dpad_right){
                frontLeft.setPower(-STRAFE_SPEED);
                frontRight.setPower(STRAFE_SPEED);
                backLeft.setPower(STRAFE_SPEED);
                backRight.setPower(-STRAFE_SPEED);

                if(!gamepad1.dpad_right){
                    break;
                }

            }

            idle();
        }
    }

}
