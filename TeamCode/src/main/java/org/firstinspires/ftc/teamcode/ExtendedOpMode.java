package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

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

    public void strafe(String direction, double power) {
        if (direction == "left") {
            robot.leftFront.setPower(power);
            robot.leftBack.setPower(-power);
            robot.rightFront.setPower(-power);
            robot.rightBack.setPower(power);
        } else if (direction == "right") {
            robot.leftFront.setPower(-power);
            robot.leftBack.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightBack.setPower(-power);
        }
    }

}
