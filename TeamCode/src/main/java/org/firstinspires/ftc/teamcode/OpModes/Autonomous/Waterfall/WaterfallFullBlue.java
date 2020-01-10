package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Autonomous(name = "Fall Blue Full", group = "Waterfall")
public class WaterfallFullBlue extends ExtendedLinearOpMode {

    public void runOpMode() {


        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initDetector();
        initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        long programStartTime = System.currentTimeMillis();
        long programEndTime = programStartTime + 30000;

        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);

        strafeDistSensor(26, Direction.RIGHT, robot.distanceLeft, 4000);

        long startTime = System.currentTimeMillis();
        long endTime = startTime + 8000;

        while (System.currentTimeMillis() < endTime && !skyStoneIsVisible("blue")) {

            setPower(0.4,0.4);
        }

        SkystonePosition skystonePosition = getSkystonePosition(robot.distanceBackRight.getDistance(DistanceUnit.INCH), Direction.BACKWARD);

        stopMotors();

        if (!skyStoneIsVisible("blue")){

            wallAlign(0.8, 48, robot.distanceBackRight, Direction.BACKWARD, 5000);
            skystonePosition = SkystonePosition.RIGHT;
        }


        //collect skystone
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        encoderDrive(40, 40, .7, .7, 5.5);
        wallAlign(0.8, 50, robot.distanceFrontLeft, Direction.FORWARD, 5000);

        //deposit skystone
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);

        encoderDrive(-80, -80, .7, .7, 5.5);
        goToStone(skystonePosition, "blue");

        //collect skystone
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.backClawCollect.setPosition(0);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);

        encoderDrive(40, 40, .7, .7, 5.5);
        wallAlign(0.8, 50, robot.distanceFrontLeft, Direction.FORWARD, 5000);

        //deposit skystone
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);

        if (skystonePosition == SkystonePosition.RIGHT) {
            skystonePosition = SkystonePosition.LEFT;
        } else if (skystonePosition == SkystonePosition.CENTER) {
            skystonePosition = SkystonePosition.RIGHT;
        } else if (skystonePosition == SkystonePosition.LEFT) {
            skystonePosition = SkystonePosition.CENTER;
        }

        if ((programEndTime - System.currentTimeMillis()) > 12) {
            encoderDrive(-80, -80, .7, .7, 5.5);
            goToStone(skystonePosition, "blue");

            //collect skystone
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
            sleep(750);
            //robot.backClawCollect.setPosition(0);
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);

            encoderDrive(40, 40, .7, .7, 5.5);
            wallAlign(0.8, 50, robot.distanceFrontLeft, Direction.FORWARD, 5000);

            //deposit skystone
            robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
            robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
        }

        gyroTurn(-90, 0.3, 3.5);

        encoderDrive(-5, -5, 0.8,0.8,1.5);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        gyroTurn(-180, 0.3, 5);

        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        encoderDrive(5, 5, 0.8,0.8,1.5);

        strafeDistSensor(26, Direction.LEFT, robot.distanceRight, 4000);

        encoderDrive(55, 55, 0.6,0.6,1.5);


    }


    }

