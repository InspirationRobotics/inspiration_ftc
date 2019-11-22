package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;

/**
 * Created by rishi on 2019-10-13
 *
 * Paraguay TeleOp
 */

@TeleOp(name = "Puddle TeleOP", group = "Puddle")
public class Paraguay extends ExtendedOpMode {

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
        setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        if (gamepad1.left_trigger > 0.2) {
            strafe("left", gamepad1.left_trigger);
        }
        if (gamepad1.right_trigger > 0.2) {
            strafe("right", gamepad1.left_trigger);
        }

        if (gamepad1.right_bumper) {

        }
        if (gamepad1.left_bumper) {

        }

    }

}
