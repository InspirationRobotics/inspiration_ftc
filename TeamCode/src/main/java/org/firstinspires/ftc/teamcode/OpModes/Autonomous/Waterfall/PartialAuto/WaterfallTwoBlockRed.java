package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Fall Two Block Red", group = "Waterfall")
public class WaterfallTwoBlockRed extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();


        //strafe right until close to stone
        strafeDistSensor(22 , Direction.RIGHT , robot.distanceLeft, 6000);

        //collect stone
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        //strafe left until in lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move backwards until middle of foundation
        wallAlign(0.8, -45, robot.distanceFront, Direction.FORWARD, 5000);

        //strafe right until close to foundation (lane one (right lane))
        strafeDistSensor(28 , Direction.RIGHT , robot.distanceLeft, 6000);

        //deposit stone
        robot.backPivot.setPosition(0);
        robot.backClawCollect.setPosition(.5);

        //strafe left until in lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move forward until second stone
        encoderDrive(150, 150, .7, .7, 6);
        wallAlign(0.8, 8, robot.distanceBack, Direction.BACKWARD, 5000);

        //strafe right to lane 1 (right lane)
        strafeDistSensor(28 , Direction.RIGHT , robot.distanceLeft, 6000);

        //collect stone
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        //strafe left until lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move backward until back of foundation
        encoderDrive(-70, -70, .7, .7, 6);
        wallAlign(0.8, 45 , robot.distanceFront, Direction.FORWARD, 5000);

        //strafe right until lane 1 (right lane)
        strafeDistSensor(28 , Direction.RIGHT , robot.distanceLeft, 6000);

        //deposit stone
        robot.backPivot.setPosition(0);
        robot.backClawCollect.setPosition(.5);

        //strafe left until lane 1 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move forward to park
        wallAlign(0.8, 68, robot.distanceBack, Direction.BACKWARD, 5000);

    }
}


