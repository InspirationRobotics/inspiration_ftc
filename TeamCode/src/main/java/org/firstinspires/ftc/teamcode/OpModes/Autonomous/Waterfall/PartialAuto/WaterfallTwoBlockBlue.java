package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Disabled
@Autonomous(name = "Fall 2 Block Blue", group = "Waterfall")
public class WaterfallTwoBlockBlue extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        //strafe right until close to stone
        strafeDistSensor(26 , Direction.RIGHT , robot.distanceLeft, 6000);

        //collect stone
        robot.backClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        //strafe left until in lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move forward until middle of foundation
        wallAlign(0.8, 45, robot.distanceFront, Direction.FORWARD, 5000);

        //strafe right until close to foundation (lane one (right lane))
        strafeDistSensor(26 , Direction.RIGHT , robot.distanceLeft, 6000);

        //deposit stone
        robot.backPivot.setPosition(0);
        robot.backClawCollect.setPosition(.5);

        //strafe left until in lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move backwards until second stone
        encoderDrive(-150, -150, .7, .7, 6000);
        wallAlign(0.8, 8, robot.distanceBack, Direction.BACKWARD, 5000);

        //strafe right to lane 1 (right lane)
        strafeDistSensor(26 , Direction.RIGHT , robot.distanceLeft, 6000);

        //collect stone
        robot.backClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        //strafe left until lane 2 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move forward until back of foundation
        encoderDrive(150, 70, .7, .7, 6);
        wallAlign(0.8, 45 , robot.distanceFront, Direction.FORWARD, 5000);

        //strafe right until lane 1 (right lane)
        strafeDistSensor(26 , Direction.RIGHT , robot.distanceLeft, 6000);

        //deposit stone
        robot.backPivot.setPosition(0);
        robot.backClawCollect.setPosition(.5);

        //strafe left until lane 1 (left lane)
        strafeDistSensor(2 , Direction.LEFT , robot.distanceLeft, 6000);

        //move backwards to park
        wallAlign(0.8, -68, robot.distanceBack, Direction.BACKWARD, 5000);

    }
}

