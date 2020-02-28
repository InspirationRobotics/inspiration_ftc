package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;

/**
 * Created by rishi on 2019-10-13
 *
 * PuddleTeleOp TeleOp
 */

@TeleOp(name = "Puddle TeleOp", group = "Puddle")
@Disabled
public class PuddleTeleOp extends ExtendedOpMode {

    @Override
    public void init () {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();
        robot.initParaguayFoundationMover();
        robot.initTilter();
    }

    @Override
    public void init_loop () {
        telemetry.addLine("Initialization successful.");
        telemetry.update();
    }

    @Override
    public void loop () {
        setPower();

        if (gamepad1.left_trigger > 0.2) {
            //strafe(gamepad1.left_trigger, 0);
        }
        if (gamepad1.right_trigger > 0.2) {
            //strafe(0, gamepad1.right_trigger);
        }

        if (gamepad1.right_bumper) {
            robot.leftCollector.setPower(1);
            robot.rightCollector.setPower(-1);
        }
        if (gamepad1.left_bumper) {
            robot.leftCollector.setPower(-1);
            robot.rightCollector.setPower(1);
        }

        robot.tilter.setPower(-gamepad2.right_stick_y);

        if(gamepad1.x) {
            robot.foundationFront.setPosition(0.1);
            robot.foundationBack.setPosition(0.9);
        }

        if (gamepad1.y) {
            robot.foundationFront.setPosition(0.9);
            robot.foundationBack.setPosition(0.1);
        }

        if (gamepad2.right_bumper) {
            robot.autoArm.setPosition(0.5);
        }

        if (gamepad2.left_bumper) {
            robot.autoArm.setPosition(1);
        }

        robot.leftCollector.setPower(0);
        robot.rightCollector.setPower(0);
    }

}
