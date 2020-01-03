package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name = "Fall Blue Full", group = "Waterfall")
public class WaterfallFullBlue extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);
        initDetector();
        //robot.initIMU();
        //initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        long startTime = System.currentTimeMillis();
        long endTime = startTime + 10000;

        while (System.currentTimeMillis() < endTime && !skyStoneIsVisible("blue")) {

            setPower(0.4,0.4);
        }

        double endDist = robot.distanceBack.getDistance(DistanceUnit.INCH) + 16;

        stopMotors();

        if (!skyStoneIsVisible("blue")){

            wallAlign(0.8, 48, robot.distanceBack, Direction.BACKWARD);

        }

        strafeDistSensor(26, Direction.RIGHT, robot.distanceLeft, 4000);

        //collect skystone
        robot.rightPivot.setPosition(.5);
        robot.rightClawCollect.setPosition(.5);
        robot.rightClawCollect.setPosition(0);
        robot.rightPivot.setPosition(0);

        wallAlign(0.8, 58, robot.distanceBack, Direction.FORWARD);

        //deposit skystone
        robot.rightPivot.setPosition(0);
        robot.rightClawCollect.setPosition(.5);

        encoderDrive(80, 80, -.7, -.7, 5.5);

        wallAlign(0.8, endDist, robot.distanceBack, Direction.BACKWARD);

        //collect skystone #2
        robot.rightPivot.setPosition(.5);
        robot.rightClawCollect.setPosition(.5);
        robot.rightClawCollect.setPosition(0);
        robot.rightPivot.setPosition(0);

        //return to position in order to deposit
        encoderDrive(80, 80, .7, .7, 5.5);

        wallAlign(0.8, 50, robot.distanceBack, Direction.FORWARD);

        //deposit skystone
        robot.rightPivot.setPosition(0);
        robot.rightClawCollect.setPosition(.5);

        gyroTurn(-90, 0.7, 2);

        robot.leftFoundation.setPosition(0.5);
        robot.rightFoundation.setPosition(0.5);

        gyroTurn(-90, 0.7, 2);

        robot.leftFoundation.setPosition(0);
        robot.rightFoundation.setPosition(0);

        encoderDrive(50, 50, -.7, -.7, 5.5);


    }


    }

