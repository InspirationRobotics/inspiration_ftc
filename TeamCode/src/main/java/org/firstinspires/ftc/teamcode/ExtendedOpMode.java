package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.util.logging.XMLFormatter;

/**
 * Created by rishi on 2019-10-13
 *
 * Functions for TeleOp
 */

public abstract class ExtendedOpMode extends OpMode {

    public Robot robot = new Robot();

    public void setPower(double left_power, double right_power) {
        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(left_power);
        robot.leftBack.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightBack.setPower(right_power);
    }

    public void strafe(double leftTrigger, double rightTrigger) {

        while (gamepad1.left_trigger > 0.2) {

            robot.leftFront.setPower(leftTrigger);
            robot.leftBack.setPower(-leftTrigger);
            robot.rightFront.setPower(-leftTrigger);
            robot.rightBack.setPower(leftTrigger);


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

            robot.leftFront.setPower(-rightTrigger);
            robot.leftBack.setPower(rightTrigger);
            robot.rightFront.setPower(rightTrigger);
            robot.rightBack.setPower(-rightTrigger);


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

    public void moveDpad(boolean gp1LeftDpad, boolean gp1RightDpad,boolean gp1UpDpad,boolean gp1DownDpad) {

        // move robot @ 50 percent power based off gp1 dpad inputs, if you couldn't read exactly one line above

        while (gp1LeftDpad) {

            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(-0.5);
            robot.rightFront.setPower(-0.5);
            robot.rightBack.setPower(0.5);


            if (!gp1LeftDpad) {
                break;
            }
        }

        while (gp1RightDpad) {

            robot.leftFront.setPower(-0.5);
            robot.leftBack.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.rightBack.setPower(-0.5);

            if (!gp1RightDpad) {
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
        robot.rightLift.setPower(gp2LeftJoystick);
    }


    public void foundationMover(boolean gp2X, boolean gp2Y) {

        // move foundation mover @ touch of x or y. x = open, y = grab fn
        // you're so smart!
        if (gp2X) {
            robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
            robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);
        } else if (gp2Y) {
            robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
            robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);
        }
    }

    /*public void extendDepositor(boolean gp2DpadLeft, boolean gp2DpadRight) {
        if (gp2DpadLeft) {
            robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_COMPACTED);
            robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_COMPACTED);
        } else if (gp2DpadRight) {
            robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_EXTENDED);
            robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_EXTENDED);
        }
    }*/ //Temporary comment becuase we are changing the extension to a different button waiting for Aditya to come back
    //same as the function a whole 12 lines down.

    public void grabBlock (boolean gp2LeftBumper, boolean gp2RightBumper) {
        // grab the block. Left = not grab. RIght = do grab. Yeet
        if (gp2LeftBumper) {
            robot.grabber.setPosition(robot.constants.GRABBER_OPEN_POS);
        } else if (gp2RightBumper) {
            robot.grabber.setPosition(robot.constants.GRABBER_GRAB_POS); // else if is used if the action cna be run at the same time // no reason why using the else if function in this case
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

    public void capstone (boolean gp2LeftDpad, boolean gp2RightDpad){
        if (gp2LeftDpad) {
            robot.capstone.setPosition(robot.constants.CAPSTONE_DOWN);
        }
        if (gp2RightDpad){
            robot.capstone.setPosition(robot.constants.CAPSTONE_UP);
        }
    }
}
