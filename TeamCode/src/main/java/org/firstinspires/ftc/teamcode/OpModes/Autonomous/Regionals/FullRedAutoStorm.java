package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "Full Auto Red Storm", group = "Storm")
public class FullRedAutoStorm extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        AllianceSide allianceSide = AllianceSide.RED;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();
        initIMU(hardwareMap);
        robot.distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        verifyBackDS(32);

        initArm();

        telemetry.addData("BACK DS", robot.distanceBack.getDistance(DistanceUnit.INCH));
        telemetry.addLine("Ready To Go!");
        telemetry.update();

        waitForStart();

        setIMUOffset();

        //moveToSkystoneStorm(2, allianceSide);
        moveToSkystoneStormBasic(2,allianceSide );
        moveToFoundationStorm(2);

        multipleStoneStormDistAlign(5);
        moveToFoundationStorm(5);

        multipleStoneStormDistAlign(1);
        moveToFoundationStorm(1);

        compactAutoArmStorm();
    }
}
