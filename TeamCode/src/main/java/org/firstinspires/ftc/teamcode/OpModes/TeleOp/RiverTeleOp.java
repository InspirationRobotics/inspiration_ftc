package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@TeleOp(name = "River TeleOp", group = "River")
public class RiverTeleOp extends ExtendedOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER_SIMPLE);
        robot.capstone = hardwareMap.servo.get("capstone");
        telemetry.addLine("Ready to go!");
        telemetry.update();
    }


    @Override
    public void loop() {
        //strafe(gamepad1.left_trigger, gamepad1.right_trigger);
        setPower();
        collect(gamepad1.left_bumper, gamepad1.right_bumper);
        moveDpad(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
        //extend(gamepad2.left_stick_y);
        //extend(gamepad2.right_stick_y);
        robot.extension.setPower(gamepad2.right_stick_y);
        //lift(gamepad2.left_stick_y);
        robot.leftLift.setPower(gamepad2.left_stick_y);
        robot.rightLift.setPower(gamepad2.left_stick_y);
        //foundationMover(gamepad2.x, gamepad2.y);
        //extendDepositor(gamepad2.dpad_left, gamepad2.dpad_right);
        grabBlock(gamepad2.left_bumper, gamepad2.right_bumper);
        //grabExtension(gamepad2.dpad_up, gamepad2.dpad_down);
        capstone(gamepad2.dpad_left, gamepad2.dpad_right);
    }


    @Override
    public void stop() {

    }
}


