package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "autoPaths")
public class AutoZeroRing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final Servo wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        final DcMotorEx wobbleGoal = hardwareMap.get(DcMotorEx.class, "wobbleGoal");

        double OPEN_POS = 0.1;
        double GRAB_POS = 0.9;

        double OUT_POWER = -0.7;
        double IN_POWER = 0.7;

        wobbleServo.setPosition(GRAB_POS);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0,29.5,0));

        Trajectory toRingStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(28, 28))
                .build();

        Trajectory toDropZoneOne_WobbleGoalOne = drive.trajectoryBuilder(toRingStack.end())
                .splineTo(new Vector2d(80, 21), 0)
                .build();

        Trajectory collectWobbleGoal = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalOne.end())
                .lineToLinearHeading(new Pose2d(25.5, 13, Math.toRadians(145)))
                .addDisplacementMarker(8, () -> {
                    wobbleGoal.setPower(OUT_POWER);
                })
                .addDisplacementMarker(40, () -> {
                    wobbleGoal.setPower(0);
                })
                .build();

        Trajectory toDropZoneOne_WobbleGoalTwo = drive.trajectoryBuilder(collectWobbleGoal.end())
                .lineToLinearHeading(new Pose2d(76, 16, Math.toRadians(0)))
                .build();


        drive.followTrajectory(toRingStack);

        sleep(2000);

        drive.followTrajectory(toDropZoneOne_WobbleGoalOne);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(collectWobbleGoal);

        wobbleServo.setPosition(GRAB_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(toDropZoneOne_WobbleGoalTwo);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(1000);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);
    }

    public void moveMotorSec(DcMotorEx motor, double power, long runTime) {
        ElapsedTime elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        while(opModeIsActive() && (elapsedTime.milliseconds() < runTime)) {
            motor.setPower(power);
        }

        motor.setPower(0);
    }
}

