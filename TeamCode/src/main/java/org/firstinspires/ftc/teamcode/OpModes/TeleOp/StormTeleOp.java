package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ExtendedOpMode;

@TeleOp(name = "Storm TeleOp", group = "Storm")
public class StormTeleOp extends ExtendedOpMode {

    public DcMotor park;

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        telemetry.addLine("Hardware Map Set (1/3)");
        telemetry.update();

        robot.initStormDrivebase();
        telemetry.addLine("Drivebase Initialized (2/3)");
        telemetry.update();

        robot.initStormAttachments();
        park = hardwareMap.dcMotor.get("park");
        telemetry.addLine("Attachments Initialized (3/3)");
        telemetry.addLine("Ready to go!");
        telemetry.update();
    }


    @Override
    public void loop() {
        setPower();
        lift();
        claw(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x);
        strafeFast();
        dpad_move();
        foundationMover(gamepad1.b, gamepad1.a);
        autoArm();
    }

    @Override
    public void stop() {

    }

//    public void park() {
//       if (gamepad2.dpad_up) {
//           park.setPower(1);
//       } else if (gamepad2.dpad_down) {
//           park.setPower(-1);
//       }
//
//       else {
//           park.setPower(0);
//       }
//    }

    public void autoArm() {
        if (gamepad2.dpad_down) {
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_DOWN_POSITION);
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_DOWN_POSITION);
        } else if (gamepad2.dpad_up) {
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_UP_POSITION+0.03);
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_UP_POSITION);
        } else if (gamepad2.dpad_left) {
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
        } else if (gamepad2.dpad_right) {
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_GRAB_POSITION);
        }
    }
}



