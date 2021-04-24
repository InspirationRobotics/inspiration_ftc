package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.SkystoneDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous
public class AutoFinalNoCollect extends LinearOpMode {
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

        /* distance sensors */
        final Rev2mDistanceSensor rbl = hardwareMap.get(Rev2mDistanceSensor.class, "RBL");
        final Rev2mDistanceSensor rbt = hardwareMap.get(Rev2mDistanceSensor.class, "RBT");
        final Rev2mDistanceSensor rfl = hardwareMap.get(Rev2mDistanceSensor.class, "RFL");
        final Rev2mDistanceSensor rft = hardwareMap.get(Rev2mDistanceSensor.class, "RFT");

        double OPEN_POS = 0.1;
        double GRAB_POS = 0.9;

        double OUT_POWER = -0.7;
        double IN_POWER = 0.7;

        wobbleServo.setPosition(GRAB_POS);
        shooterTilt.setPosition(0);

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
            wobbleGoalPos[0] = 57;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 4) {
            wobbleGoalPos[0] = 105;
            wobbleGoalPos[1] = 1;
        } else if (numberOfRings == 1) {
            wobbleGoalPos[0] = 81;
            wobbleGoalPos[1] = 21;
        }

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        telemetry.addLine("started");

        drive.setPoseEstimate(new Pose2d(0,29.5,0));

        Trajectory toRingStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(-2)))
                .addDisplacementMarker(8, () -> {
                    shooterTilt.setPosition(0);
                })
                .build();

        Trajectory toDropZoneOne_WobbleGoalOne = drive.trajectoryBuilder(toRingStack.end())
                .lineToLinearHeading(new Pose2d(wobbleGoalPos[0], wobbleGoalPos[1], 0))
                .build();

        Trajectory collectWobbleGoal = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalOne.end())
                .lineToLinearHeading(new Pose2d(24, 12, Math.toRadians(155)))
//                .addDisplacementMarker(8, () -> {
//                    wobbleGoal.setPower(OUT_POWER);
//                })
//                .addDisplacementMarker(40, () -> {
//                    wobbleGoal.setPower(0);
//                })
                .build();

        Trajectory toDropZoneOne_WobbleGoalTwo = drive.trajectoryBuilder(toRingStack.end())
                .lineToLinearHeading(new Pose2d(wobbleGoalPos[0]-12, wobbleGoalPos[1]-3, 0))
                .build();

        Trajectory park = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalTwo.end())
                .strafeTo(new Vector2d(70, 21))
                .build();;

        if (numberOfRings == 0) {
            park = drive.trajectoryBuilder(toDropZoneOne_WobbleGoalTwo.end())
                    .strafeTo(new Vector2d(46, 22))
                    .build();
        }

        Trajectory finalize_park_zero_rings = drive.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(70, 33))
                .build();

        drive.followTrajectory(toRingStack);

        shooterTilt.setPosition(0);
        shooterOne.setVelocity(-196, AngleUnit.DEGREES);
        shooterTwo.setVelocity(-196, AngleUnit.DEGREES);
        sleep(1000);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);
        sleep(1000);
        magazine.setPosition(0.5);
        sleep(1000);
        magazine.setPosition(1);

        shooterOne.setVelocity(0, AngleUnit.DEGREES);
        shooterTwo.setVelocity(0, AngleUnit.DEGREES);


        drive.followTrajectory(toDropZoneOne_WobbleGoalOne);

        moveMotorSec(wobbleGoal, OUT_POWER, 1500);
        wobbleServo.setPosition(OPEN_POS);
        sleep(500);
        //moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(collectWobbleGoal);

        wobbleServo.setPosition(GRAB_POS);
        sleep(800);
        moveMotorSec(wobbleGoal, IN_POWER, 1500);

        drive.followTrajectory(toDropZoneOne_WobbleGoalTwo);

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

