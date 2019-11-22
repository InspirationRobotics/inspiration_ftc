package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;

/**
 * Created by rishi on 2019-10-13
 *
 * Paraguay TeleOp
 */

@TeleOp(name = "Paraguay", group = "Outreach")
public class Paraguay extends ExtendedOpMode {

    @Override
    public void init () {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivebase();
    }

    @Override
    public void init_loop () {
        telemetry.addLine("Initialization successful.");
        telemetry.update();
    }

    @Override
    public void loop () {
        setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }

}
