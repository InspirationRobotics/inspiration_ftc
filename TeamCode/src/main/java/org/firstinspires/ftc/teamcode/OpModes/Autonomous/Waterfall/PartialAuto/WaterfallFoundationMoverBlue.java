package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Waterfall.PartialAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.RobotVersion;

@Autonomous(name = "Fall Foundation Mover Blue", group = "Waterfall")
public class WaterfallFoundationMoverBlue extends ExtendedLinearOpMode {

    public void runOpMode() {

        robot.setHardwareMap(hardwareMap);
        robot.initRiver(RobotVersion.RIVER);
        initDetector();
        //robot.initIMU();
        //initIMU(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        //moving backwards to align with foundation
        wallAlign(0.8, 40, robot.distanceFront, Direction.BACKWARD);

        //grab foundation
        robot.leftFoundation.setPosition(robot.constants.LEFT_FOUNDATION_GRAB_POS);
        robot.rightFoundation.setPosition(robot.constants.RIGHT_FOUNDATION_GRAB_POS);

        //turning clockwise 90 degrees to move foundation into
        gyroTurn(90, 0.7, 2);

        //move forward to park
        wallAlign(0.8, 58, robot.distanceBack, Direction.FORWARD);

    }
}

