package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "Full Auto Blue Storm", group = "Storm")
public class FullBlueAutoStorm extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        AllianceSide allianceSide = AllianceSide.BLUE;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();
        initIMU(hardwareMap);
        robot.distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");


        initArm();

        telemetry.addLine("Ready To Go!");
        telemetry.update();

        waitForStart();

        setIMUOffset();

        moveToSkystoneStorm(2, allianceSide);
        moveToFoundationStorm(2);

        multipleStoneStorm(5);
        moveToFoundationStorm(5);

        multipleStoneStorm(1);
        moveToFoundationStorm(1);

        compactAutoArmStorm();
    }
}
