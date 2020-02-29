package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.sql.Time;
import java.util.logging.XMLFormatter;

/**
 * Created by rishi on 2019-10-13
 *
 * Functions for TeleOp
 */

public abstract class ExtendedOpMode extends OpMode {

    public Robot robot = new Robot();

    public void setPower() {
        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(gamepad1.left_stick_y);
        robot.leftBack.setPower(gamepad1.left_stick_y);
        robot.rightFront.setPower(gamepad1.right_stick_y);
        robot.rightBack.setPower(gamepad1.right_stick_y);
    }

    public void lift() {
        robot.lift.setPower(gamepad2.left_stick_y);
    }

    public void claw(boolean open, boolean close, boolean mid) {

        if (open)
            robot.claw.setPosition(0);
        if (close)
            robot.claw.setPosition(1);
        if (mid)
            robot.claw.setPosition(0.6);

    }

    public void dpad_move() {
        while(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {

            double dp_d = -(gamepad1.dpad_down ? 1 : 0) * 0.5;
            double dp_u = -(gamepad1.dpad_up ? 1 : 0) * 0.5;
            double dp_l = (gamepad1.dpad_left ? 1 : 0) * 0.5;
            double dp_r = (gamepad1.dpad_right ? 1 : 0) * 0.5;

            robot.leftFront.setPower(dp_u - dp_d + -dp_r + dp_l);
            robot.leftBack.setPower(dp_u - dp_d + dp_r - dp_l);
            robot.rightFront.setPower(dp_u - dp_d + dp_r - dp_l);
            robot.rightBack.setPower(dp_u - dp_d + -dp_r + dp_l);

        }
    }

    public void strafeFast() {

        while(gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2) {
            double dp_l = (gamepad1.left_trigger > 0.2 ? 1 : 0);
            double dp_r = (gamepad1.right_trigger > 0.2 ? 1 : 0);
            robot.leftFront.setPower(-dp_r + dp_l);
            robot.leftBack.setPower(dp_r - dp_l);
            robot.rightFront.setPower(dp_r - dp_l);
            robot.rightBack.setPower(-dp_r + dp_l);
        }
    }
   /*public void strafe(double leftTrigger, double rightTrigger) {

       while (gamepad1.left_trigger > 0.2) {

           robot.leftFront.setPower(gamepad1.left_trigger);
           robot.leftBack.setPower(-gamepad1.left_trigger);
           robot.rightFront.setPower(-gamepad1.left_trigger);
           robot.rightBack.setPower(gamepad1.left_trigger);


           if (gamepad1.left_trigger <= 0.2) {
               robot.leftFront.setPower(0);
               robot.leftBack.setPower(0);
               robot.rightFront.setPower(0);
               robot.rightBack.setPower(0);
               break;
           }

           //it strfafes and escapes the loop so tht it doesn't run forever
           // matched for kept and right because logically, you should be able to move left
       }

       while (gamepad1.right_trigger > 0.2) {

           robot.leftFront.setPower(-gamepad1.right_trigger);
           robot.leftBack.setPower(gamepad1.right_trigger);
           robot.rightFront.setPower(gamepad1.right_trigger);
           robot.rightBack.setPower(-gamepad1.right_trigger);


           if (gamepad1.right_trigger <= 0.2) {
               robot.leftFront.setPower(0);
               robot.leftBack.setPower(0);
               robot.rightFront.setPower(0);
               robot.rightBack.setPower(0);
               break;
           }
       }

       //it strfafes and escapes theloop so tht it doesn't run forever
       // matched for keft and right bcecause lkgogivally, you dhould be able to move left

   }*/

    public void moveAutoArm(){
        if(gamepad2.dpad_left){
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        } else if (gamepad2.dpad_right){
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
        } else if (gamepad2.dpad_down){
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        } else if (gamepad2.dpad_up) {
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_MID);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
        }
    }

    public void moveAutoArmCopy() {
         if (gamepad2.dpad_up) {
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_MID);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
        } else if (gamepad2.dpad_down){

             if (robot.backClawCollect.getPosition() != robot.constants.BACK_CLAW_COLLECT_GRAB)
                 robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
             if (robot.frontClawCollect.getPosition() != robot.constants.BACK_CLAW_COLLECT_GRAB)
                 robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);
            robot.frontPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        }
    }

    public void collect(boolean leftBumper, boolean rightBumper)  {
        //Left is outtake, right is intake. (they are the bumpers on gp1)
        //Power 1 intakes
        //Power -1 outtakes

        if(leftBumper) {
            robot.intake.setPower(-1);
        } else if (rightBumper) {
            robot.intake.setPower(1);
        } else {
            robot.intake.setPower(0);
        }
    }

    public void collectWaterfall(boolean leftBumper, boolean rightBumper) {
        //Left is outtake, right is intake. (they are the bumpers on gp1)
        //Power 1 intakes
        //Power -1 outtakes

        if(leftBumper) {
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(1);
        } else if (rightBumper) {
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(-1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void moveDpad(boolean gp1LeftDpad, boolean gp1RightDpad,boolean gp1UpDpad,boolean gp1DownDpad) {

        // move robot @ 50 percent power based off gp1 dpad inputs, if you couldn't read exactly one line above

        while (gamepad1.dpad_left) {

            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(-0.5);
            robot.rightFront.setPower(-0.5);
            robot.rightBack.setPower(0.5);


            if (!gamepad1.dpad_left) {
                break;
            }
        }

        while (gamepad1.dpad_right) {

            robot.leftFront.setPower(-0.5);
            robot.leftBack.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.rightBack.setPower(-0.5);

            if (!gamepad1.dpad_right) {
                break;
            }
        }

        while (gamepad1.dpad_down) {

            robot.leftFront.setPower(-0.5);
            robot.leftBack.setPower(-0.5);
            robot.rightFront.setPower(-0.5);
            robot.rightBack.setPower(-0.5);

            if (!gamepad1.dpad_down) {
                break;
            }
        }

        while (gamepad1.dpad_up) {

            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.rightBack.setPower(0.5);

            if (!gamepad1.dpad_up) {
                break;
            }
        }
    }

    public void extend(double gp2RightJoystick) {
        //move horizontal extension of collector the speed of gp2 right joystick y. thank you for attendindg my Ted Talk
//      // if the limit switch is being not being triggered . . .
//        if (!robot.extensionLimit.getState())
////            robot.extension.setPower(gp2RightJoystick);
        robot.extension.setPower(gp2RightJoystick);
    }

    public void lift(double gp2LeftJoystick) {
        //move the vertical lift @ speed of gp2 left joystick
        robot.leftLift.setPower(gp2LeftJoystick);
    }


    public void foundationMover(boolean open, boolean close) {

        if (open) {
            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);
        } else if (close) {
            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);
        }
    }

    public void foundationMoverFall(boolean gp2X, boolean gp2Y) {

        // move foundation mover @ touch of x or y. x = open, y = grab fn
        // you're so smart!
        if (gp2X) {
            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);
        } else if (gp2Y) {
            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);
        }
    }

  /* public void extendDepositor(boolean gp2DpadLeft, boolean gp2DpadRight) {
       if (gp2DpadLeft) {
           robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_COMPACTED);
           robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_COMPACTED);
       } else if (gp2DpadRight) {
           robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_EXTENDED);
           robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_EXTENDED);
       }
   }*/ //Temporary comment becuase we are changing the extension to a different button waiting for Aditya to come back
    //same as the function a whole 12 lines down. sample change

    public void grabBlock (boolean gp2LeftBumper, boolean gp2RightBumper) {
        // grab the block. Left = not grab. RIght = do grab. Yeet
        if (gp2LeftBumper) {

            robot.grabber.setPosition(robot.constants.GRABBER_OPEN_POS); //1
        } else if (gp2RightBumper) {
            robot.grabber.setPosition(robot.constants.GRABBER_GRAB_POS); //0

            //robot.grabber.setPosition(robot.grabber.getPosition() - 0.1);
            // else if is used if the action cna be run at the same time // no reason why using the else if function in this case
        }
    }

    public void grabExtension (boolean gp2UpBumper, boolean gp2DownBumper){
       //based off pressing the gp2 clicky bumper, extend if left pressed. COmpact if right pressed. oof
       if (gp2UpBumper){
           robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_EXTENDED);
           robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_EXTENDED);
       }
       if (gp2DownBumper){
           robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_COMPACTED);
           robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_COMPACTED);
       }
   }

    public void horizantalExtend (){
        if (gamepad2.right_stick_y > 0.2) {
            robot.extensionMotor.setPower(1);
        } else if (gamepad2.right_stick_y < 0.2) {
            robot.extensionMotor.setPower(-1);
        } else {
            robot.extensionMotor.setPower(0);
        }
    }

    public void capstone (boolean gp2X, boolean gp2Y){
        if (gp2X) {
            robot.capstone.setPosition(robot.constants.CAPSTONE_DOWN);
        }
        if (gp2Y){
            robot.capstone.setPosition(robot.constants.CAPSTONE_UP);
        }
    }


}

