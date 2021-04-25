package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous
public class AutoFinalCollectRings extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        /** control hub */

        /* distance sensors */
        final Rev2mDistanceSensor lfl = hardwareMap.get(Rev2mDistanceSensor.class, "LFL");
        final Rev2mDistanceSensor lft = hardwareMap.get(Rev2mDistanceSensor.class, "LFT");
        final Rev2mDistanceSensor lbl = hardwareMap.get(Rev2mDistanceSensor.class, "LBL");
        final Rev2mDistanceSensor lbt = hardwareMap.get(Rev2mDistanceSensor.class, "LBT");

        /** expansion hub */

        /* motors */
        final DcMotorEx wobbleGoal = hardwareMap.get(DcMotorEx.class, "wobbleGoal");
        final DcMotorEx shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        final DcMotorEx shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        final DcMotorEx collector = hardwareMap.get(DcMotorEx.class, "collector");

        /* servos */
        final Servo wobbleServo = hardwareMap.get(Servo.class, "servoWobbleGoal");
        final Servo shooterTilt = hardwareMap.get(Servo.class, "shooterTilt");
        final Servo magazine = hardwareMap.get(Servo.class, "magazine");
        final Servo whisker = hardwareMap.get(Servo.class, "whisker");

        /* distance sensors */
        final Rev2mDistanceSensor rbl = hardwareMap.get(Rev2mDistanceSensor.class, "RBL");
        final Rev2mDistanceSensor rbt = hardwareMap.get(Rev2mDistanceSensor.class, "RBT");
        final Rev2mDistanceSensor rfl = hardwareMap.get(Rev2mDistanceSensor.class, "RFL");
        final Rev2mDistanceSensor rft = hardwareMap.get(Rev2mDistanceSensor.class, "RFT");

        double OPEN_POS = 0.1;
        double GRAB_POS = 0.9;

        double OUT_POWER = -0.7;
        double IN_POWER = 0.7;

        double TILT_UP_POS = 0.4;
        double TILT_DOWN_POS = 1;

        wobbleServo.setPosition(GRAB_POS);
        shooterTilt.setPosition(TILT_UP_POS);

        OpenCvInternalCamera phoneCam;
        SkystoneDeterminationPipeline pipeline;
        int numberOfRings;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.pauseViewport();

        phoneCam.openCameraDevice();
        phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while (!isStarted()) {
            telemetry.addData("ringnum", pipeline.returnNum());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        numberOfRings = pipeline.returnNum();
        telemetry.addData("final ring scan", numberOfRings);
        telemetry.update();

        double[] wobbleGoalPos = {80, 21};
        if (numberOfRings == 0) {
            wobbleGoalPos[0] = 59;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 4) {
            wobbleGoalPos[0] = 107;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 1) {
            wobbleGoalPos[0] = 83;
            wobbleGoalPos[1] = 21;
        }

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        telemetry.addLine("started");

        drive.setPoseEstimate(new Pose2d(0,29.5,0));

        Trajectory toRingStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(-2)))
                .addDisplacementMarker(1, () -> {
                    wobbleGoal.setPower(OUT_POWER);
                    collector.setPower(-1);
                    shooterOne.setVelocity(-196, AngleUnit.DEGREES);
                    shooterTwo.setVelocity(-196, AngleUnit.DEGREES);
                })
                .addDisplacementMarker(14, () -> {
                    wobbleGoal.setPower(0);
                })
                .build();

//        Trajectory collectRing = drive.trajectoryBuilder(toRingStack.end())
//                .forward(20)
//                .build();

        Trajectory strafeAvoidRing = drive.trajectoryBuilder(toRingStack.end())
                .strafeLeft(26)
                .build();

        Trajectory driveToWGOne_4Rings = drive.trajectoryBuilder(strafeAvoidRing.end())
                .lineToLinearHeading(new Pose2d(wobbleGoalPos[0], wobbleGoalPos[1], 0))
                .addDisplacementMarker(6, () -> {
                    shooterOne.setVelocity(0, AngleUnit.DEGREES);
                    shooterTwo.setVelocity(0, AngleUnit.DEGREES);
                })
                .build();


        Trajectory toDropZoneOne_WobbleGoalOne = drive.trajectoryBuilder(toRingStack.end())
                .lineToLinearHeading(new Pose2d(wobbleGoalPos[0], wobbleGoalPos[1], 0))
                .addDisplacementMarker(6, () -> {
                    if(numberOfRings == 1 ) {
                        collector.setPower(1);
                    }
                    shooterOne.setVelocity(0, AngleUnit.DEGREES);
                    shooterTwo.setVelocity(0, AngleUnit.DEGREES);
                })
                .build();

        Trajectory returnToLaunchPos = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalOne.end())
                .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(-2)))
                .addDisplacementMarker(6, () -> {
                    shooterOne.setVelocity(-196, AngleUnit.DEGREES);
                    shooterTwo.setVelocity(-196, AngleUnit.DEGREES);
                })
                .build();

        Trajectory forward_forCollectWG = drive.trajectoryBuilder(returnToLaunchPos.end())
                .strafeTo(new Vector2d(32, 4))
                .build();

        Trajectory collectWobbleGoal = drive.trajectoryBuilder(numberOfRings == 0 ? toDropZoneOne_WobbleGoalOne.end() : forward_forCollectWG.end())
                .lineToLinearHeading(new Pose2d(26, 12, Math.toRadians(175)))
//                .addDisplacementMarker(8, () -> {
//                    wobbleGoal.setPower(OUT_POWER);
//                })
//                .addDisplacementMarker(40, () -> {
//                    wobbleGoal.setPower(0);
//                })
                .build();

        Trajectory toDropZoneOne_WobbleGoalTwo = drive.trajectoryBuilder(collectWobbleGoal.end())
                .lineToLinearHeading(new Pose2d(wobbleGoalPos[0]-9, wobbleGoalPos[1]-3, 0))
                .build();

        Trajectory park = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalTwo.end())
                .strafeTo(new Vector2d(70, 21))
                .build();

        if (numberOfRings == 0) {
            park = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalTwo.end())
                    .strafeTo(new Vector2d(46, 22))
                    .build();
        }

        Trajectory finalize_park_zero_rings = drive.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(70, 33))
                .build();

        if (numberOfRings == 1) {
            collector.setPower(-1);
        } else if (numberOfRings == 4) {
            collector.setPower(-0.5);
        }

        drive.followTrajectory(toRingStack);

        collector.setPower(0);

        shooterOne.setVelocity(-196, AngleUnit.DEGREES);
        shooterTwo.setVelocity(-196, AngleUnit.DEGREES);

        shooterTilt.setPosition(TILT_UP_POS);
        sleep(700);
        magazine.setPosition(1);
        sleep(700);
        magazine.setPosition(0.5);
        sleep(700);
        magazine.setPosition(1);
        sleep(700);
        magazine.setPosition(0.5);
        sleep(700);
        magazine.setPosition(1);
        sleep(700);
        magazine.setPosition(0.5);
        sleep(700);

        shooterTilt.setPosition(TILT_DOWN_POS);

        if(numberOfRings == 1) {

            drive.followTrajectory(toDropZoneOne_WobbleGoalOne);


            collector.setPower(1);
            wobbleServo.setPosition(OPEN_POS);
            sleep(1000);

            collector.setPower(1);
            drive.followTrajectory(returnToLaunchPos);

            shooterTilt.setPosition(TILT_UP_POS);
            shooterOne.setVelocity(-196, AngleUnit.DEGREES);
            shooterTwo.setVelocity(-196, AngleUnit.DEGREES);

            collector.setPower(0);

            magazine.setPosition(1);

            int numLoop = 0;

            if (numberOfRings == 1) {
                numLoop = 2;
            } else {
                numLoop = 4;
            }

            for(int i = 0; i < numLoop; i++) {
                magazine.setPosition(0.5);
                sleep(700);
                magazine.setPosition(1);
                sleep(700);
            }

            shooterOne.setVelocity(0, AngleUnit.DEGREES);
            shooterTwo.setVelocity(0, AngleUnit.DEGREES);
        } else {

            shooterOne.setVelocity(0, AngleUnit.DEGREES);
            shooterTwo.setVelocity(0, AngleUnit.DEGREES);

            if (numberOfRings == 4) {
                collector.setPower(-0.5);
            } else {
                collector.setPower(0);
            }


            if(numberOfRings == 0) {
                drive.followTrajectory(toDropZoneOne_WobbleGoalOne);
            } else {
                drive.followTrajectory(strafeAvoidRing);
                drive.followTrajectory(driveToWGOne_4Rings);
            }

            collector.setPower(0);

            moveMotorSec(wobbleGoal, OUT_POWER, 1500);
            wobbleServo.setPosition(OPEN_POS);
            sleep(750);
            //moveMotorSec(wobbleGoal, IN_POWER, 1500);
        }

        drive.followTrajectory(forward_forCollectWG);

        drive.followTrajectory(collectWobbleGoal);

        wobbleServo.setPosition(GRAB_POS);
        sleep(800);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(toDropZoneOne_WobbleGoalTwo);

        whisker.setPosition(0.2);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(500);
        moveMotorSec(wobbleGoal, IN_POWER, 1000);

        drive.followTrajectory(park);

        if(numberOfRings == 0) {
            drive.followTrajectory(finalize_park_zero_rings);
        }
//        drive.followTrajectory(park);
    }

    public void moveMotorSec(DcMotorEx motor, double power, long runTime) {
        ElapsedTime elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        while (opModeIsActive() && (elapsedTime.milliseconds() < runTime)) {
            motor.setPower(power);
        }

        motor.setPower(0);
        //I feel like a bug
    }
}

