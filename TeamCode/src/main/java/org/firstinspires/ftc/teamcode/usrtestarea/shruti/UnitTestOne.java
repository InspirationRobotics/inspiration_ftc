package org.firstinspires.ftc.teamcode.usrtestarea.shruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonAutoFunctions;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Forward/Backward")
public class UnitTestOne extends CommonAutoFunctions{

    Robot robot   = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        hwit();
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initAllServos();
        robot.initMiscMotors();

        telemetry.addData("Heading", globalHeading);
        telemetry.addData("Status", globalCoordinates[0]);
        telemetry.addData("Status", globalCoordinates[1]);
        telemetry.update();
        waitForStart();

        wobbleDrop();

        /* int count = 1;

       while (count <= 20) {



          encoderDriveByInches(30, 0.2, 10, robot.frontLeft.getCurrentPosition(),
                   robot.frontRight.getCurrentPosition(),
                   robot.backLeft.getCurrentPosition(),
                   robot.backRight.getCurrentPosition());
           runtime.reset();
           while (opModeIsActive() && (runtime.seconds() < 1)) {

           }
           ;

           encoderDriveByInches(-30, 0.2, 10, robot.frontLeft.getCurrentPosition(),
                   robot.frontRight.getCurrentPosition(),
                   robot.backLeft.getCurrentPosition(),
                   robot.backRight.getCurrentPosition());
           runtime.reset();
           while (opModeIsActive() && (runtime.seconds() < 1)) {

           }

           count = count +1;*/
       }
        }