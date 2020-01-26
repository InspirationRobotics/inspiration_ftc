package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Foundation Align")
public class FoundationAlign extends ExtendedLinearOpMode {

    public void runOpMode(){
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        //move to foundation
        encoderDrive(50,50,0.8, 0.8, 5000);

        wallAlign(0.6, 18, robot.distanceRight, Direction.RIGHT, 5000);
        wallAlign(0.6, 30, robot.distanceFront, Direction.FORWARD, 5000);
    }
}
