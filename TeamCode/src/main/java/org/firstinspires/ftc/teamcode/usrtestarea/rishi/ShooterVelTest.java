package org.firstinspires.ftc.teamcode.usrtestarea.rishi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Autonomous(name="Shooter Vel Test", group="Simple Auto")
public class ShooterVelTest extends CommonAutoFunctions {

    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        hwit();
        robot.setHardwareMap(hardwareMap);
        robot.initMiscMotors();
        robot.initAllServos();

        waitForStart();

        robot.shooterOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double motorVel = -1470;
//        robot.shooterOne.setVelocity(motorVel);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {}
        while (opModeIsActive()) {
            robot.shooter.setPosition(0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
            }
            robot.shooter.setPosition(0.6);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
            }
        }

    }
}
