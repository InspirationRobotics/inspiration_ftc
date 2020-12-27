package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
@Disabled
public class Rings extends MovementHelperFunctions {

    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        int[][] start = {
                {60,90}
        };

        int[][] positions = {
                {40, 72},
                {50, 72},
                {60, 72},
        };

        /* init hw */
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initAllDistanceSensors();
        robot.initMiscMotors();

        telemetry.addData("Status", "ready");
        telemetry.update();
        waitForStart();

        // travel the vertical distance between the start and the first position
        encoderDriveByInches(-(start[1][0] - positions[1][0]), -(start[1][0] - positions[1][0]), 0.5, 10);
        encoderTurn(-90, 0.5, 10);

        // travel the horizontal distance
        encoderDriveByInches((start[0][0] - positions[0][0]), (start[0][0] - positions[0][0]), 0.5, 10);
        encoderTurn(90, 0.5, 10);

        /* now is the time to fire */

        encoderTurn(90, 0.5, 10);
        encoderDriveByInches((positions[1][0] - positions[0][0]), (positions[1][0] - positions[0][0]), 0.5, 10);
        encoderTurn(-90, 0.5, 10);

        /* now is the time to fire */

        encoderTurn(90, 0.5, 10);
        encoderDriveByInches((positions[2][0] - positions[1][0]), (positions[2][0] - positions[1][0]), 0.5, 10);
        encoderTurn(-90, 0.5, 10);

        /* now is the time to fire */

    }
}
