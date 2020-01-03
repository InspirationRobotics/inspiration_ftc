package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;

@Autonomous(name = "Fall Red Full", group = "Waterfall")
public class  WaterfallFullRed extends ExtendedLinearOpMode {

    public void runOpMode() {


        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        initDetector();
        //robot.initIMU();
        //initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        //record end time for program for time sensitive tasks like additional skystone collection
        long programStartTime = System.currentTimeMillis();
        long programEndTime = programStartTime + 30000;

        //move into the lane next to the skystones
        strafeDistSensor(26, Direction.RIGHT, robot.distanceLeft, 4000);

        //sweep forward to look for skystones
        long startTime = System.currentTimeMillis();
        long endTime = startTime + 8000;

        while (System.currentTimeMillis() < endTime && !skyStoneIsVisible("red")) {

            setPower(0.4,0.4);
        }

        SkystonePosition skystonePosition = getSkystonePosition(robot.distanceFront.getDistance(DistanceUnit.INCH), Direction.FORWARD);

        stopMotors();

        //failsafe if skystone is not detected. Grab the middle stone the far side from wall
        if (!skyStoneIsVisible("red")){

            wallAlign(0.8, 48, robot.distanceFront, Direction.FORWARD);
            skystonePosition = SkystonePosition.RIGHT;
        }


        //collect skystone
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.rightClawCollect.setPosition(0);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        //drive to the build zone
        encoderDrive(-40, -40, .7, .7, 5.5);
        //align to be a certain distance to put block on foundation
        wallAlign(0.8, 50, robot.distanceBack, Direction.BACKWARD);

        //deposit skystone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_OPEN);
        sleep(750);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        encoderDrive(80, 80, .7, .7, 5.5);
        goToStone(skystonePosition, "red");

        //collect skystone
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
        sleep(750);
        //robot.rightClawCollect.setPosition(0);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        encoderDrive(-40, -40, .7, .7, 5.5);
        wallAlign(0.8, 50, robot.distanceBack, Direction.BACKWARD);

        //deposit skystone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_OPEN);
        sleep(750);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

        //go get another set of stones
        if (skystonePosition == SkystonePosition.RIGHT) {
            skystonePosition = SkystonePosition.LEFT;
        } else if (skystonePosition == SkystonePosition.CENTER) {
            skystonePosition = SkystonePosition.RIGHT;
        } else if (skystonePosition == SkystonePosition.LEFT) {
            skystonePosition = SkystonePosition.CENTER;
        }

        //if there's enough time, we'll go for the third stone
        if ((programEndTime - System.currentTimeMillis()) > 12) {
            encoderDrive(80, 80, .7, .7, 5.5);
            goToStone(skystonePosition, "red");

            //collect skystone
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
            sleep(750);
            //robot.rightClawCollect.setPosition(0);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);

            encoderDrive(-40, -40, .7, .7, 5.5);
            wallAlign(0.8, 50, robot.distanceBack, Direction.BACKWARD);

            //deposit skystone
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_OPEN);
            sleep(750);
            robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
            robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);
        }

        //turn so the back of robot with foundation movers faces the long side of foundation
        gyroTurn(-90, 0.3, 3.5);

        //drive back to touch foundation
        encoderDrive(-5, -5, 0.8,0.8,1.5);

        //grab foundation
        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        //turn to move foundation into the magical floor dorito® (build zone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
        gyroTurn(0, 0.3, 5);

        //ungrab the foundation (yeet)
        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_OPEN_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_OPEN_POS);

        //drive forward for fun (and to not be on the foundation)
        encoderDrive(5, 5, 0.8,0.8,1.5);

        //make sure we are in the inside lane to park, becuase parking is so much fun and is my desire to live
        strafeDistSensor(26, Direction.RIGHT, robot.distanceLeft, 4000);

        //PPPPPPPPPAAAAAAAAAAARRRRRRRRRKKKKKKKKKKK (WE PARK YEET)
        encoderDrive(55, 55, 0.6,0.6,1.5);


    }


}
