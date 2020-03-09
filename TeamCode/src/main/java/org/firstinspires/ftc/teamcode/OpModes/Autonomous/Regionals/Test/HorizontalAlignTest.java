package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;

@Autonomous(name = "HorizontalAlignTest", group = "Storm")
public class HorizontalAlignTest extends BasicExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        AllianceSide allianceSide = AllianceSide.BLUE;
        alliance = AllianceSide.BLUE;
        robot.setHardwareMap(hardwareMap);
        robot.initStormDrivebase();
        robot.initStormAttachments();
        initIMU(hardwareMap);
        robot.distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        robot.distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        robot.distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
        robot.distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
        verifyBackDS(32);


        //initArm();

        telemetry.addData("BACK DS", robot.distanceBack.getDistance(DistanceUnit.INCH));
        telemetry.addLine("Ready To Go!");
        telemetry.update();

        setIMUOffset();

        waitForStart();

        wallAlignHorizontally(robot.constants.WALL_DIST_STONE, 0.7, allianceSide);

        gyroTurn(0,0.4,2);
        sleep(250);
    }
}
