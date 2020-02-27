package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;

@TeleOp(name = "Storm TeleOp", group = "Storm")
public class StormTeleOp extends ExtendedOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        telemetry.addLine("Hardware Map Set (1/3)");
        telemetry.update();

        robot.initStormDrivebase();
        telemetry.addLine("Drivebase Initialized (2/3)");
        telemetry.update();

        robot.initStormAttachments();
        telemetry.addLine("Attachments Initialized (3/3)");
        telemetry.addLine("Ready to go!");
        telemetry.update();
    }


    @Override
    public void loop() {
        setPower();
        lift();
        claw();
        dpad_move();
        foundationMover(gamepad1.right_trigger, gamepad1.left_trigger);
    }

    @Override
    public void stop() {

    }
}



