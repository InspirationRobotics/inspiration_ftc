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

        while (leftTrigger > 0.2) {

            robot.leftFront.setPower(leftTrigger);
            robot.leftBack.setPower(-leftTrigger);
            robot.rightFront.setPower(-leftTrigger);
            robot.rightBack.setPower(leftTrigger);


            if (leftTrigger <= 0.2) {
                break;
            }
        }



        while (rightTrigger > 0.2) {

            robot.leftFront.setPower(rightTrigger);
            robot.leftBack.setPower(-rightTrigger);
            robot.rightFront.setPower(-rightTrigger);
            robot.rightBack.setPower(rightTrigger);


            if (rightTrigger <= 0.2) {
                break;
            }
        }

    }

    public void collect(boolean leftBumper, boolean rightBumper) {
        //Left is outtake, right is intake.
        //Power 1 inttakes
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

    public void extend(double gp2LeftJoystick) {
        robot.extension.setPower(gp2LeftJoystick);
    }

    public void lift(double gp2RightJoystick) {
        robot.leftLift.setPower(gp2RightJoystick);
        robot.rightLift.setPower(gp2RightJoystick);
    }

    public void foundationMover(boolean gp2X, boolean gp2Y) {
        if (gp2X) {
            robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
            robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);
        } else if (gp2Y) {
            robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
            robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);
        }
    }

    public void extendDepositor(boolean gp2DpadLeft, boolean gp2DpadRight) {
        if (gp2DpadLeft) {
            robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_COMPACTED);
            robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_COMPACTED);
        } else if (gp2DpadRight) {
            robot.leftExtension.setPosition(robot.constants.LEFT_EXTENSION_EXTENDED);
            robot.rightExtension.setPosition(robot.constants.RIGHT_EXTENSION_EXTENDED);
        }
    }

    public void grabBlock (boolean gp2LeftBumper, boolean gp2RightBumper) {
        if (gp2LeftBumper) {
            robot.grabber.setPosition(robot.constants.GRABBER_OPEN_POS);
        } else if (gp2RightBumper) {
            robot.grabber.setPosition(robot.constants.GRABBER_GRAB_POS);
        }
    }
}
