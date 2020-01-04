package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@TeleOp(name = "Fall TeleOp", group = "Waterfall")
public class WaterfallTeleOp extends ExtendedOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        telemetry.addLine("Ready to go!");
        telemetry.update();
    }


    @Override
    public void loop() {
        setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);
        collectWaterfall(gamepad1.left_bumper, gamepad1.right_bumper);
        moveDpad(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
        robot.leftLift.setPower(-gamepad2.left_stick_y);
        robot.rightLift.setPower(gamepad2.left_stick_y);
        foundationMoverFall(gamepad2.x, gamepad2.y);
        extendDepositor(gamepad2.dpad_left, gamepad2.dpad_right);
        //horizantalExtend(); .
        robot.extensionServo.setPower(-gamepad2.right_stick_y);
        grabBlock(gamepad2.left_bumper, gamepad2.right_bumper);
        moveAutoArm();
    }

    @Override
    public void stop() {

    }
}



