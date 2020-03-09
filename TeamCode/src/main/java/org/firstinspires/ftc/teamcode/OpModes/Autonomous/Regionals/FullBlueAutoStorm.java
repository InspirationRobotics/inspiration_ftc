package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Regionals;

import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BasicExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Autonomous(name = "Full Auto Blue Storm", group = "Storm")
public class FullBlueAutoStorm extends BasicExtendedLinearOpMode {

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
        DcMotor park = hardwareMap.get(DcMotor.class,"park");
        verifyBackDS(32);

        setInitLiftPosition();

        initArm();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

        telemetry.addData("BACK DS", robot.distanceBack.getDistance(DistanceUnit.INCH));
        telemetry.addLine("Ready To Go!");
        telemetry.update();

        waitForStart();

        double endVisionTime = System.currentTimeMillis() + 3000;

        while((System.currentTimeMillis() < endVisionTime) && opModeIsActive() && (detector.skystoneId(AllianceSide.BLUE) == SkystonePosition.UNKNOWN)) {
            idle();
        }

        SkystonePosition skystonePosition = detector.skystoneId(AllianceSide.BLUE);
        int skystoneId;


        if (skystonePosition == SkystonePosition.LEFT) {
            skystoneId = 1;
        } else if (skystonePosition == SkystonePosition.CENTER) {
            skystoneId = 2;
        } else if (skystonePosition == SkystonePosition.RIGHT) {
            skystoneId = 3;
        } else {
            skystoneId = 2;
        }

        setIMUOffset();

        //moveToSkystoneStorm(2, allianceSide);
        moveToSkystoneStormRegionals(skystoneId, allianceSide);
//        moveToFoundationStormDist(2, 80);
        moveToFoundationStormDistAlignRegionals(skystoneId, 2);

        multipleStoneStormDistAlignRegionals(skystoneId + 3);
//        moveToFoundationStormDist(5, 84);
        moveToFoundationStormDistAlignRegionals(skystoneId + 3, 6);

//        multipleStoneStormDistAlign(1);
//        moveToFoundationStorm(1);

        compactAutoArmStorm();
        moveFoundationStorm(AllianceSide.BLUE);

        park.setPower(1);

        gyroDrive(0.4,15,38);
        while (opModeIsActive()) {
            park.setPower(1);
        }
        park.setPower(0);
        //moveLift(initialLiftPosition);
    }
}
