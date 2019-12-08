package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name="Puddle Auto Red", group = "Puddle")
public class PuddleAutoRed extends ExtendedLinearOpMode {


    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        initPuddle(hardwareMap);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();

        long startTime = System.currentTimeMillis();
        long endTime = startTime + 1000;

        while (System.currentTimeMillis() < endTime) {
            robot.tilter.setPower(1);
        }

        robot.leftCollector.setPower(0);

        startTime = System.currentTimeMillis();
        endTime = startTime + 350;

        while (System.currentTimeMillis() < endTime) {
            robot.tilter.setPower(-1);
        }

        robot.leftCollector.setPower(0);
        //positive one for both motors is intake

        startTime = System.currentTimeMillis();
        endTime = startTime + 2500;

        while (System.currentTimeMillis() < endTime) {
            robot.leftFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightFront.setPower(1);
            robot.rightBack.setPower(1);
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        sleep(1000);

        robot.leftCollector.setPower(0);
        robot.rightCollector.setPower(0);



    }

    void intake() {

    }

    void outtake() {

    }
}
