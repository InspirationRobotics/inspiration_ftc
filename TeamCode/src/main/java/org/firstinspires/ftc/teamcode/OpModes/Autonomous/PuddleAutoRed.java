package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name="Puddle Auto Red", group = "Puddle")
public class PuddleAutoRed extends ExtendedLinearOpMode {

    Robot robot = new Robot();

    public void runOpMode() {

        initPuddle(hardwareMap);

        telemetry.addLine("Initialized! Ready to go!");
        telemetry.update();

        waitForStart();


    }

    void intake() {

    }

    void outtake() {

    }
}
