package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test.AxesSigns;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test.BNO055IMUUtil;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

public abstract class BasicExtendedLinearOpMode extends LinearOpMode {
    public Robot robot = new Robot();
    public SkystoneDetector detector = new SkystoneDetector();
    private ElapsedTime runtime = new ElapsedTime();
    public AllianceSide alliance = AllianceSide.RED;

    public double initialIMUOffset = 0;

    public boolean useDistanceSensorFront = true;
    public boolean useDistanceSensorBack = true;

    public double initialLiftPosition = 0;

    static final double     P_DRIVE_COEFF           = 0.085;     // Larger is more responsive, but also less stable


    /* imu functions */

    public void initIMU(HardwareMap ahawmp) {

        robot.imu = hardwareMap.get(BNO055IMU.class, robot.constants.IMU_NAME);

        //Initialize IMU parameters
        robot.parameters = new BNO055IMU.Parameters();
        robot.parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        robot.parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        robot.parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        robot.parameters.loggingEnabled      = true;
        robot.parameters.loggingTag          = "IMU";
        robot.parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(robot.parameters);

//        BNO055IMUUtil.remapAxes(robot.imu, AxesOrder.ZYX, AxesSigns.NNN);

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);

        for (int i = 0; i<10; i++)
            getHeading();

        initialIMUOffset = robot.angles.firstAngle;

        robot.imu.initialize(robot.parameters);

        telemetry.addData("imu heading", getHeading());

    }

    public void strafeGyro(double speed, double maintainedAngle) {

        setMotorRunMode(RUN_WITHOUT_ENCODER);

        double powerLeftBack = 0.8*speed;
        double powerLeftFront = -0.8*speed;
        double powerRightBack = -0.8*speed;
        double powerRightFront = 0.8*speed;

        double error = getError(maintainedAngle);
        double error_proportioned = Range.clip((error*0.08), -0.2, 0.2);

        powerLeftBack = powerLeftBack - error_proportioned;
        powerLeftFront = powerLeftFront - error_proportioned;
        powerRightBack = powerRightBack + error_proportioned;
        powerRightFront = powerRightFront + error_proportioned;

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);
    }

    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);

        if(robot.DEBUG) System.out.println("11128dbg Heading = " + robot.angles);

        return robot.angles.firstAngle-initialIMUOffset;
    }

    public void gyroTurn(double targetAngle, double speed, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        while (opModeIsActive() && (System.currentTimeMillis() < endTime) && !onHeading(speed, targetAngle, robot.constants.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
            telemetry.update();
        }

        stopMotors();
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.constants.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = -speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180 && opModeIsActive())  robotError -= 360;
        while (robotError <= -180 && opModeIsActive()) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        if(error > 0) {
            return Range.clip(error * PCoeff, 0.225, 1);
        } else {
            return Range.clip(error * PCoeff, -1, -0.225);
        }
    }

    public double getSteerDrive(double error, double PCoeff) {
            return Range.clip(error * PCoeff, -0.25, 0.25);
    }
    /* movement */

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
    }

    public void moveToSkystone(int skystoneId, AllianceSide allianceSide) {

        gyroTurn(0,0.5,1);

        if (allianceSide == AllianceSide.BLUE) {
            encoderStrafeGyro(robot.constants.WALL_DIST_STONE, 1, 0, 3);
        } else {
            encoderStrafeGyro(robot.constants.WALL_DIST_STONE, 1, 0, 3);
        }
        gyroTurn(0,0.5,2);

        double targetDistance;
        DistanceSensor wallAlignSensor;
        Direction wallAlignDirection;

        if (allianceSide == AllianceSide.BLUE) {
            targetDistance = -(8*skystoneId)+18.5;
        } else {
            if (skystoneId < 5) {
                targetDistance = (8 * skystoneId) - 12;
            } else {
                targetDistance = (8 * 4) - 16;
            }
        }

        encoderDrive(targetDistance, targetDistance, 0.5, 0.5, 3500);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        grabAutoArm();
    }

    /* auto arm */

    public void grabAutoArm(){
        //grab stone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        sleep(700);
        double endTime = System.currentTimeMillis() + 325;
        while (System.currentTimeMillis() < endTime) {
            strafeGyro(1, 0);
        }

        stopMotors();

        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
        sleep(500);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
        endTime = System.currentTimeMillis() + 325;
        while (System.currentTimeMillis() < endTime) {
            strafeGyro(-1, 0);
        }
    }

    public void releaseAutoArm() {
        //release stone
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        sleep(400);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_OPEN);
        robot.frontPivot.setPosition((robot.constants.FRONT_PIVOT_MID+robot.constants.FRONT_PIVOT_UP)/2);
    }

    /* movement functions */

    public void encoderStrafe(double units, double speed) {

        units = units*0.68;
        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(STOP_AND_RESET_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            // setPowerStrafe(Math.abs(speed), Math.abs(speed));
            setPower(Math.abs(speed), Math.abs(speed));

            while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", units);
                telemetry.addData("Target Distance Right (in)", units);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }
        }
    }

    public void setPower (double left_power, double right_power) {

        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(-left_power);
        robot.rightFront.setPower(-right_power);
        robot.rightBack.setPower(-right_power);
        robot.leftBack.setPower(-left_power);
    }

    public void setPowerReverse (double left_power, double right_power) {

        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */
        robot.rightBack.setPower(-right_power);
        robot.leftBack.setPower(-left_power);
        robot.leftFront.setPower(-left_power);
        robot.rightFront.setPower(-right_power);
    }

    public void setTargetPositionStrafe(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightFront.getCurrentPosition() + leftTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + rightTarget);
        robot.rightFront.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void encoderDriveBasicGyro(double distance, double speed, double angle, double timeoutS) {
        int leftTarget = -(int)(robot.constants.STRAFE_TICKS_PER_IN*distance);
        int rightTarget = (int)(robot.constants.STRAFE_TICKS_PER_IN*distance);

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        setTargetPositionStrafe(leftTarget,rightTarget);

        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double leftPower;
        double rightPower;
        double error;
        double steer;

        runtime.reset();

        while(robot.rightBack.isBusy() && robot.leftFront.isBusy() && (runtime.seconds() < timeoutS)) {

            double powerLeftBack = 0.9*speed;
            double powerLeftFront = -0.9*speed;
            double powerRightBack = -0.9*speed;
            double powerRightFront = 0.9*speed;

            error = getError(angle);
            double error_proportioned = -Range.clip((error*0.055), -0.3, 0.3);

            powerLeftBack = powerLeftBack + error_proportioned;
            powerLeftFront = powerLeftFront + error_proportioned;
            powerRightBack = powerRightBack - error_proportioned;
            powerRightFront = powerRightFront - error_proportioned;

            robot.leftFront.setPower((powerLeftFront));
            robot.rightFront.setPower((powerRightFront));
            robot.leftBack.setPower((powerLeftBack));
            robot.rightBack.setPower((powerRightBack));


        }

        stopMotors();

        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderStrafeBasic(double distance, double speed, double timeoutS) {
        int leftTarget = -(int)(robot.constants.STRAFE_TICKS_PER_IN*distance);
        int rightTarget = (int)(robot.constants.STRAFE_TICKS_PER_IN*distance);

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        setTargetPositionStrafe(leftTarget,rightTarget);

        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double leftPower;
        double rightPower;

        runtime.reset();

        while(robot.rightBack.isBusy() && robot.leftFront.isBusy() && (runtime.seconds() < timeoutS)) {

            setPower(speed,speed);
        }

        stopMotors();

        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //



    public void stopWheels() {
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setPower(0);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setPower(0);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setPower(0);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setPower(0);
    }

    public void vectorCombineSimple(double x, double y, double turn) {


        double a = (x + y) + turn;
        double b = (y - x) - turn;
        double c = -(y - x) - turn;
        double d = -(x + y) + turn;

        robot.leftFront.setPower(a);
        robot.rightFront.setPower(b);
        robot.leftBack.setPower(c);
        robot.rightBack.setPower(d);

        if(DEBUG) System.out.println("111128bg vectorCombineSimple a " + a + " b " + b + " c " + c + " d " + d );

    }


    public void EncoderStrafe(double dist){
        EncoderMoveDist(1, dist,true, false, 0);
    }

    public void EncoderStrafeGyro(double dist){
        EncoderMoveDist(1, dist,true, true, 0);
    }

    boolean DEBUG = true;
    double  TICKS_PER_INCH_STRAFE= robot.constants.STRAFE_TICKS_PER_IN;
    double  TICKS_PER_INCH_STRAIGHT= TICKS_PER_INCH_STRAFE /  robot.constants.STRAFE_CONSTANT ;
    double running_counter = 0;
    double where_head;

    double P_FWD_COEFF = -0.003;
    double FWD_THRESHOLD = 1;

    public void EncoderMoveDist(double speed, double distance, Boolean strafe, Boolean gyroCorrection, double sideWays) {

        if(DEBUG) System.out.println("11128dbg EncoderMoveDist: " + distance + "Starfe: " + strafe);

        where_head = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;

        distance *= -1;

        robot.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        robot.leftBack.setMode(STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(RUN_WITHOUT_ENCODER);

        double target_encoder = 0;

        if(!strafe) {
            target_encoder = (distance * TICKS_PER_INCH_STRAIGHT);
            //speed *= 0.9;
        }
        else {
            target_encoder = (distance * TICKS_PER_INCH_STRAFE);
        }

        double prev_pos = robot.rightBack.getCurrentPosition();

        //first pass at high speed (if going large dist)
        int stall_counter = 0;
        double sidePwr = sideWays;
        if((Math.abs(target_encoder) > TICKS_PER_INCH_STRAIGHT*10) || (strafe)){
        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF, TICKS_PER_INCH_STRAIGHT, strafe, gyroCorrection, sidePwr)) {
            double currPosition = robot.rightBack.getCurrentPosition();
            if (prev_pos == currPosition) {
                stall_counter++;
            } else {
                stall_counter = 0;
                prev_pos = currPosition;
            }
            if (stall_counter > 10)
                break;

            if(Math.abs(currPosition) > Math.abs(target_encoder*.3))
                sidePwr = sideWays*-1;
            idle();
        }
        }


        //second pass at low speed for fine granined distance
        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF/2 , TICKS_PER_INCH_STRAIGHT/4, strafe, gyroCorrection, sidePwr)) {
            if (prev_pos == robot.rightBack.getCurrentPosition()) {
                stall_counter++;
            }
            else {
                stall_counter = 0;
                prev_pos = robot.rightBack.getCurrentPosition();
            }
            if(stall_counter > 10)
                break;

            idle();
        }

    }


    boolean onTargetDist(double speed, double distance, double PCoeff, double distThreshold, Boolean strafe, Boolean gyroCorrection, double sideWays) {
        double error;
        double steer;
        boolean onTarget = false;
        double power;

        //determine  power based on error
        error = getErrorDist(distance, strafe, distThreshold);

        if (Math.abs(error) <= distThreshold) {

            steer = 0.0;
            power = 0.0;
            sideWays = 0.0;
            onTarget = true;
            stopWheels();
        } else {

            steer = getSteerDist(error, PCoeff);
            power = speed * steer;
        }

        double weightConstant = 0.98;//this constant will depend on the robot. you need to test experimentally to see which is best


        double turn_pwr = 0;
        if(gyroCorrection) {
            if ((running_counter++ % 3) == 0) {
                double curr_angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
                if(curr_angle < -90)
                    curr_angle += 360;
                turn_pwr = (where_head - curr_angle) * 0.1;


                if (DEBUG)
                    System.out.println("11128dbg where_head: " + where_head + " " + where_head);
                if (DEBUG) System.out.println("11128dbg curr_angle: " + curr_angle);

                turn_pwr = turn_pwr * -1;

                if (DEBUG) System.out.println("11128dbg turn_pwr: " + turn_pwr);

            }
        }

        if(strafe) {
            power = power*2;
            while (Math.abs(weightConstant * power) < 0.3)
                weightConstant *= 1.2;
        }
        else {
            while (Math.abs(weightConstant * power) < 0.2)
                weightConstant *= 1.2;
        }


        if (!strafe) {
            double sidePwr = sideWays*weightConstant * power;
            if(sidePwr == weightConstant * power)
                sidePwr = sideWays+0.001;  //just to avoid divide by 0

            vectorCombineSimple(-sidePwr, weightConstant * power, turn_pwr);
        }
        else {
            vectorCombineSimple( weightConstant * power, 0, turn_pwr);
        }

        if(DEBUG) System.out.println("11128dbg " + " steer " + steer + " error " + error + " power " + power*weightConstant);


        telemetry.addData("Target dist", "%5.2f", distance);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f", speed);

        return onTarget;
    }

    public double getErrorDist(double targetDist, Boolean strafe, double distThr) {

        double robotError;
        double robotError2;
        double err;
        double curr_encoder = 0;
        if(!strafe)
            curr_encoder = robot.rightBack.getCurrentPosition();
        else
            curr_encoder = 1 * robot.rightBack.getCurrentPosition();

        robotError = targetDist - curr_encoder;

        if (targetDist <0) {
            robotError2 = Math.min(curr_encoder, -1*(distThr*5));
            err = Math.max(robotError, robotError2);
        }
        else {
            robotError2 = Math.max(curr_encoder, (distThr*5));
            err = Math.min(robotError, robotError2);
        }

        if(DEBUG) System.out.println("11128dbg getError curr_enc= " + curr_encoder + " err= " + err + " distThr= " + distThr);
        telemetry.addData("Robot Error", "%5.2f", robotError);
        telemetry.update();

        return err;

    }

    public double getSteerDist(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void simpleStraight(double power) {
        robot.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.leftFront.setPower(power);
        robot.rightBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);

        if(DEBUG) System.out.println("11128dbg simpleStraight power: " + power);

    }

    public void simpleStrafe(double power) {
        robot.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.leftFront.setPower(power);
        robot.rightBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);

        if(DEBUG) System.out.println("11128dbg simpleStrafe power: " + power);


    }

    //












    public void moveToSkystoneStormBasic(int skystoneId, AllianceSide alliance) {
        double targetDistance = robot.constants.WALL_DIST_STONE;

        if (alliance == AllianceSide.RED) {
            targetDistance = targetDistance*-1;
        }

        encoderDriveBasicGyro(targetDistance,1,0,3.5);

        gyroTurn(1,0.25,1.5);

        if (useDistanceSensorBack) {
            alignToStone(skystoneId);
        } else {
            double targetDriveDistance = 25 - (8 * skystoneId);

            gyroDrive(0.8, targetDriveDistance, 0);

        }
        alignHorizontally(robot.constants.WALL_DIST_STONE);
        gyroTurn(1,0.25,1.5);

        grabAutoArmStorm();
    }

    public void moveToSkystoneStormRegionals(int skystoneId, AllianceSide alliance) {

        double targetDistance = robot.constants.WALL_DIST_STONE;

        wallAlignHorizontally(targetDistance, 0.7, alliance);

        sleep(200);

//        wallAlignHorizontally(targetDistance,0.7,alliance);

        gyroTurn( 0,0.25,1.5);

        if (useDistanceSensorBack) {
            alignToStone(skystoneId);
        } else {
            double targetDriveDistance = 25 - (8 * skystoneId);

            gyroDrive(0.8, targetDriveDistance, 0);

        }
//        gyroTurn(0,0.25,1.5);
        grabAutoArmStorm();

    }

    public void setMotorRunMode(DcMotor.RunMode runmode) {
        robot.leftFront.setMode(runmode);
        robot.rightFront.setMode(runmode);
        robot.leftBack.setMode(runmode);
        robot.rightBack.setMode(runmode);
    }

    public void encoderDriveAcc(double left_in, double right_in, double speed_l, double speed_r, double timeoutS) {

        int left_distanceEnc = -(int) (robot.constants.COUNTS_PER_INCH * left_in);
        int right_distanceEnc =  -(int) (robot.constants.COUNTS_PER_INCH * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(STOP_AND_RESET_ENCODER);
//            sleep(100);

            setTargetPosition(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
//            setPower(Math.abs(speed_l), Math.abs(speed_r));

            double inputSpeed_l = 0;
            double inputSpeed_r = 0;
            double timesRun = 0;

            double accelerateRamp = 0;

            while ((opModeIsActive() && runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.rightBack.isBusy()) {



                inputSpeed_l = Range.clip((accelerateRamp),0,Math.abs(speed_l));
                inputSpeed_r = Range.clip((accelerateRamp),0,Math.abs(speed_r));

                if (left_distanceEnc < 0) {
                    setPower(Math.abs(inputSpeed_l),Math.abs(inputSpeed_r));
                } else {
                    setPowerReverse(Math.abs(inputSpeed_l),Math.abs(inputSpeed_r));
                }

                accelerateRamp = Range.clip((timesRun*timesRun)*0.005,0,1);

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", left_in);
                telemetry.addData("Target Distance Right (in)", right_in);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight05", right_distanceEnc);
                telemetry.update();
                //just one more test...

                timesRun++;
                sleep(5);
            }

            stopMotors();

            sleep(250);

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void encoderDrive(double left_in, double right_in, double speed_l, double speed_r, double timeoutS) {

        int left_distanceEnc = -(int) (robot.constants.COUNTS_PER_INCH * left_in);
        int right_distanceEnc =  -(int) (robot.constants.COUNTS_PER_INCH * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(STOP_AND_RESET_ENCODER);
//            sleep(100);

            setTargetPosition(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(Math.abs(speed_l), Math.abs(speed_r));

            while ((opModeIsActive() && runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", left_in);
                telemetry.addData("Target Distance Right (in)", right_in);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight05", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            stopMotors();

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            sleep(250);

        }

    }

    public void setTargetPosition(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + rightTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void moveToFoundation(int skystoneId, AllianceSide allianceSide) {

        double targetDriveDist = (120 - (8 * (7 + -skystoneId)));

        if (skystoneId > 4) {
            targetDriveDist = targetDriveDist + 5;
        } else {
            targetDriveDist = targetDriveDist - 3;
        }

        gyroTurn(0,0.5,2);

        switch (allianceSide) {
            case RED:
                targetDriveDist = -(targetDriveDist);
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                sleep(50);
//                double endTimeRed = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeRed) {
//                    strafeGyro(1, 0);
//                }
                encoderStrafeGyro(8,1,0,1.5);
                break;
            case BLUE:
                targetDriveDist = targetDriveDist - 16;
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                sleep(50);
//                double endTimeBlue = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeBlue) {
//                    strafeGyro(1, 0);
//                }
                encoderStrafeGyro(8,1,0,1.5);
                break;
        }

        stopMotors();
//        strafeWallDist(robot.constants.WALL_DIST_FOUNDATION,1,robot.distanceLeft, Direction.LEFT, 4000);
        gyroTurn(0, 0.5, 2);

//        releaseAutoArm();
    }

    public void multipleStones(int skystoneId, AllianceSide allianceSide) {

        double targetDriveDist = -(133.5 - (8 * (7 + -skystoneId)));

        if (skystoneId > 4) {
            targetDriveDist = targetDriveDist + 5;
        } else {
            targetDriveDist = targetDriveDist - 3;
        }

        gyroTurn(0,0.5,2);

        switch (allianceSide) {
            case RED:
//                double endTimeRed = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeRed) {
//                    strafeGyro(-1, 0);
//                }
                encoderStrafeGyro(-11,1,0,3);
                gyroTurn(0,0.5,2);
                targetDriveDist = -(targetDriveDist);
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                break;
            case BLUE:
//                double endTimeBlue = System.currentTimeMillis() + 650;
//                while (System.currentTimeMillis() < endTimeBlue) {
//                    strafeGyro(-1, 0);
//                }
                encoderStrafeGyro(-11,1,0,3);
                gyroTurn(0,0.5,2);
                targetDriveDist = targetDriveDist + 16;
                encoderDrive(targetDriveDist, targetDriveDist, 1, 1, 10);
                break;
        }

//        strafeWallDist(robot.constants.WALL_DIST_FOUNDATION,1,robot.distanceLeft, Direction.LEFT, 4000);
        gyroTurn(0, 0.5, 2);
        grabAutoArm();
    }

    public void moveToSkystoneStorm(int skystoneId, AllianceSide allianceSide) {

        double speed = 0.9;
        double distance;

        if (allianceSide == AllianceSide.BLUE) {
            distance = robot.constants.WALL_DIST_STONE;
        } else {
            distance = -robot.constants.WALL_DIST_STONE;
        }

        encoderStrafeGyro(distance, speed, 0, 3);
        gyroTurn(0,0.25,1);
        alignHorizontally(robot.constants.WALL_DIST_STONE);

        double targetDistance = 19-(8*skystoneId);

        if (useDistanceSensorBack) {
            gyroDrive(0.8,targetDistance,0);
        }
        else {
            alignToStone(skystoneId);
        }
//        encoderDrive(targetDistance, targetDistance, 0.7, 0.7, 3500);


        //grabAutoArmStorm();
    }

    public void grabAutoArmStorm() {
        long downSleepTimeMS = 820;
        long grabWaitTimeMS = 550;
        if (alliance ==AllianceSide.RED) {
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_DOWN_POSITION);
            sleep(downSleepTimeMS);
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_GRAB_POSITION);
            sleep(grabWaitTimeMS);
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_UP_POSITION);
            sleep(downSleepTimeMS);
        } else {
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_DOWN_POSITION);
            sleep(downSleepTimeMS);
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_GRAB_POSITION);
            sleep(grabWaitTimeMS);
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_UP_POSITION);
            sleep(downSleepTimeMS);
        }
    }

    public void releaseAutoArmStorm() {
        long downSleepTimeMS = 500;
        long grabWaitTimeMS = 400;
        if (alliance == AllianceSide.RED) {
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_DOWN_POSITION);
            sleep(downSleepTimeMS);
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
            sleep(grabWaitTimeMS);
            robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_UP_POSITION);
        }
        else {
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_DOWN_POSITION);
            sleep(downSleepTimeMS);
            robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
            sleep(grabWaitTimeMS);
            robot.autoPivotRight.setPosition(robot.constants.RIGHT_AUTO_PIVOT_UP_POSITION);
        }
    }

    public void compactAutoArmStorm() {
        long downSleepTimeMS = 500;
        robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_UP_POSITION);
    }

    public void initArm() {
        robot.autoPivotLeft.setPosition(robot.constants.LEFT_AUTO_PIVOT_MID_POSITION);
        robot.autoCollect.setPosition(robot.constants.AUTO_COLLECT_OPEN_POSITION);
    }

    public void moveToFoundationStorm(int skystoneId) {
        double targetDist = 80 + (8*skystoneId);

        double multiplier = 1;
        if (alliance == AllianceSide.BLUE) {
            multiplier = -1;
        }

//        gyroTurn(0,0.2,1);
        gyroDrive(1,targetDist,0.6*multiplier);
//        encoderDrive(targetDist, targetDist, 1, 1, 5);
        gyroTurn(0,0.35,0.65);

        releaseAutoArmStorm();
    }

    public void moveToFoundationStormDist(int skystoneId, int distance) {
        double targetDist = distance + (8*skystoneId);

        double multiplier = 1;
        if (alliance == AllianceSide.BLUE) {
            multiplier = -1;
        }

//        gyroTurn(0,0.2,1);
        gyroDrive(1,targetDist,0.6*multiplier);
//        encoderDrive(targetDist, targetDist, 1, 1, 5);
        gyroTurn(0,0.35,0.65);
        alignHorizontally(robot.constants.WALL_DIST_STONE);
        gyroTurn(0,0.35,0.65);

        releaseAutoArmStorm();
        sleep(500);
    }

    public void moveToFoundationStormDistAlign(int skystoneId, int wallDist) {
        double targetDist = 80 + (8*skystoneId);

        double multiplier = 1;
        if (alliance == AllianceSide.BLUE) {
            multiplier = -1;
        }

//        gyroTurn(0,0.2,1);
        gyroDrive(1,targetDist,0);
//        encoderDrive(targetDist, targetDist, 1, 1, 5);
        gyroTurn(0,0.25,1);
        alignHorizontally(robot.constants.WALL_DIST_FOUNDATION);

        verifyFrontDS(10);
        if (useDistanceSensorFront) {
            wallAlign(wallDist, 0.5, robot.distanceFront, Direction.FORWARD);
        } else if (wallDist < 20) {
            gyroDrive(0.6,14,0);
        }

        releaseAutoArmStorm();
        alignHorizontally(robot.constants.WALL_DIST_STONE);

    }

    public void moveToFoundationStormDistAlignRegionals(int skystoneId, int wallDist) {
        double targetDist = 80 + (8*skystoneId);

        gyroDrive(1,targetDist,0);
        gyroTurn(0,0.25,1);
        wallAlignHorizontally(robot.constants.WALL_DIST_FOUNDATION, 0.7, alliance);

        verifyFrontDS(10);
        if (useDistanceSensorFront) {
            wallAlign(wallDist, 0.5, robot.distanceFront, Direction.FORWARD);
        } else if (wallDist < 20) {
            gyroDrive(0.6,14,0);
        }

        releaseAutoArmStorm();
        alignHorizontally(robot.constants.WALL_DIST_STONE);
    }

    public void multipleStoneStorm(int skystoneId) {
        double targetDist = 80 + (8*skystoneId);

//        gyroTurn(0,0.2,1);
        gyroDrive(0.9,-targetDist,-0.275);
//        encoderDrive(-targetDist, -targetDist, 1, 1, 5);
        gyroTurn(-0.55,0.35,1);

        grabAutoArmStorm();
    }

    public void multipleStoneStormDistAlign(int skystoneId) {
        double targetDist = 80 + (8*skystoneId);

        double multiplier = 1;
        if (alliance == AllianceSide.BLUE) {
            multiplier = -1;
        }

        gyroTurn(0,0.2,1);
        gyroDrive(1,-targetDist,0*multiplier);
//        encoderDrive(targetDist, targetDist, 1, 1, 5);
//        gyroTurn(0,0.25,0.75);
        alignToStone(skystoneId);
        alignHorizontally(robot.constants.WALL_DIST_STONE);
        gyroTurn(0,0.25,0.75);
        grabAutoArmStorm();
    }
    public void multipleStoneStormDistAlignRegionals(int skystoneId) {
        double targetDist = 80 + (8*skystoneId);

        double multiplier = 1;
        if (alliance == AllianceSide.BLUE) {
            multiplier = -1;
        }

        gyroTurn(0,0.2,1);
        gyroDrive(1,-targetDist,0*multiplier);
        gyroTurn(0,0.3,1);
        alignToStone(skystoneId);
        alignHorizontally(robot.constants.WALL_DIST_STONE);
        gyroTurn(0,0.25,0.75);
//        wallAlignHorizontally(robot.constants.WALL_DIST_STONE, 0.7, alliance);
        grabAutoArmStorm();
    }

    public void alignToStone(int skystoneId) {
        if (useDistanceSensorBack) {
            double targetDist = 51 - (8 * skystoneId);

            wallAlign(targetDist, 1, robot.distanceBack, Direction.BACKWARD);
        }
    }
    public void wallAlign(double targetDistance, double speed, DistanceSensor inputDistanceSensor, Direction distanceSensorDirection) {
        double encoderDist = 0;

        for (int i = 0; i<4; i++)
            inputDistanceSensor.getDistance(DistanceUnit.INCH);

        double currentDist = inputDistanceSensor.getDistance(DistanceUnit.INCH);

        encoderDist = currentDist - targetDistance;

        if (distanceSensorDirection == Direction.BACKWARD) {
            encoderDist = -encoderDist;
        }

        encoderDrive(encoderDist, encoderDist, speed, speed, 3);
    }

    public void wallAlign(double targetDistance, double speed, DistanceSensor inputDistanceSensor, Direction distanceSensorDirection, double acceptableMargin) {
        double encoderDist = 0;

        for (int i = 0; i<4; i++)
            inputDistanceSensor.getDistance(DistanceUnit.INCH);

        sleep(150);

        double currentDist = inputDistanceSensor.getDistance(DistanceUnit.INCH);

        encoderDist = currentDist - targetDistance;

        if(encoderDist <3.5) {

        }
        else {
            if (distanceSensorDirection == Direction.BACKWARD) {
                encoderDist = -encoderDist;
            }

            encoderDrive(encoderDist, encoderDist, speed, speed, 6);

        }
    }

    public void moveFoundation(AllianceSide allianceSide) {

        if (allianceSide == AllianceSide.BLUE) {

            gyroTurn(-90, 0.6, 10);

            encoderDrive(-10, -10, 0.3, 0.3, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

            sleep(1000);

//            encoderDrive(-5, -5, 0.7, 0.7, 10);

            encoderDrive(22, 22, 0.8, 0.8, 10);

            gyroTurn(-180, 1, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            sleep(250);

            encoderDrive(5, 5, 1, 1, 0.8);
        }

        else if (allianceSide == AllianceSide.RED) {

//            while ((robot.distanceLeft.getDistance(DistanceUnit.INCH) > 28.5) && opModeIsActive()) {
//                strafeGyro(-1, 0);
//            }

            gyroTurn(-90, 0.6, 10);

            encoderDrive(-10, -10, 0.3, 0.3, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);

            sleep(1000);

//            encoderDrive(-5, -5, 0.7, 0.7, 10);

            encoderDrive(18, 18, 0.8, 0.8, 10);

//            sleep(250);

            gyroTurn(-180, 1, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            sleep(250);

            encoderDrive(5, 5, 1, 1, 0.8);
        }

    }

    public void moveFoundationStorm(AllianceSide allianceSide) {

        liftTime(0.5,1);

        if (allianceSide == AllianceSide.BLUE) {

            //moveLift(1200);

            gyroTurn(-90, 0.3, 10);

            //encoderDrive(20, 20, 0.6, 0.6, 10);
            gyroDrive(0.4,20,-90);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);
            sleep(500);

            gyroDrive(0.6,-42,-90);

            gyroTurn(0, 0.5, 5);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            encoderDrive(5, 5, 1, 1, 0.8);
        }

        else if (allianceSide == AllianceSide.RED) {

            //moveLift(1200);

            gyroTurn(90, 0.3, 2);

            encoderDrive(20, 20, 0.6, 0.6, 10);

            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_GRAB_POS);
            sleep(500);

            encoderDrive(-42, -42, 1, 1, 10);
            gyroTurn(0, 0.75, 10);
            robot.foundationServo.setPosition(robot.constants.FOUNDATION_SERVO_OPEN_POS);

//            encoderDrive(-5, -5, 1, 1, 0.8);
        }

        gyroDrive(1,10,0);
        liftTime(0.5,-1);
    }

    public void parkBridge(AllianceSide allianceSide) {

        compactAutoArm();

        double targetHeading;
        double strafeDist = -5;

        if (allianceSide == AllianceSide.BLUE) {
            targetHeading = -178;
        } else {
            targetHeading = 2;
            strafeDist = strafeDist*-1;
        }

        encoderStrafeGyro(strafeDist, 0.5, 0, 3);
        gyroTurn(targetHeading, 0.5, 1);
        encoderDrive(24, 24, 1, 1, 3.5);

}

    public void compactAutoArm(){
        robot.frontPivot.setPosition(0.2);
        robot.backClawCollect.setPosition(0.1);
    }

    public void encoderStrafeGyro(double units, double speed, double maintainedAngle, double timeoutS) {

        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        int speedMultiplier;

        if (units > 0) {
            speedMultiplier=1;
        } else {
            speedMultiplier=-1;
        }
        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        if (opModeIsActive()) {

            setMotorRunMode(STOP_AND_RESET_ENCODER);

            sleep(100);
            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            //setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            boolean in_threshold_lb;
            boolean in_threshold_rf;

            if (units>0) {
                in_threshold_lb = (robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition()) > 0;
                in_threshold_rf = (robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition()) < 0;
            } else {
                in_threshold_lb = (robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition()) < 0;
                in_threshold_rf = (robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition()) > 0;
            }

//            double err_lb = (robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition());
//            double err_rf = (robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition());

            boolean stayInLoop = true;

            while (opModeIsActive() && (!in_threshold_lb) && (!in_threshold_rf) && (System.currentTimeMillis() < endTime) && stayInLoop) {


//                if(units > 0 && err_rf > 0) {
//                    stayInLoop = false;
//                    break;
//                } else if(units < 0 && err_rf > 0) {
//                    stayInLoop = false;
//                    break;
//                }

//                err_lb = Math.abs(robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition());
//                err_rf = Math.abs(robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition());

                if (units>0) {
                    in_threshold_lb = (robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition()) > 0;
                    in_threshold_rf = (robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition()) < 0;
                } else {
                    in_threshold_lb = (robot.leftBack.getCurrentPosition() -robot.leftBack.getTargetPosition()) < 0;
                    in_threshold_rf = (robot.rightFront.getCurrentPosition() -robot.rightFront.getTargetPosition()) > 0;
                }

                double powerLeftBack = 0.9*speed*speedMultiplier;
                double powerLeftFront = -0.9*speed*speedMultiplier;
                double powerRightBack = -0.9*speed*speedMultiplier;
                double powerRightFront = 0.9*speed*speedMultiplier;

                double error = getError(maintainedAngle);
                double error_proportioned = -Range.clip((error*0.1), -0.2, 0.2);

                powerLeftBack = powerLeftBack - error_proportioned;
                powerLeftFront = powerLeftFront - error_proportioned;
                powerRightBack = powerRightBack + error_proportioned;
                powerRightFront = powerRightFront + error_proportioned;

                robot.leftFront.setPower((powerLeftFront));
                robot.rightFront.setPower((powerRightFront));
                robot.leftBack.setPower((powerLeftBack));
                robot.rightBack.setPower((powerRightBack));

            }
            stopMotors();
        }


    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = -(int)(distance * robot.constants.COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFront.isBusy() && robot.rightBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteerDrive(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFront.setPower(leftSpeed);
                robot.rightFront.setPower(rightSpeed);
                robot.leftBack.setPower(leftSpeed);
                robot.rightBack.setPower(rightSpeed);
            }

//            runtime.reset();
//
//            while(runtime.seconds()<0.35) {
//                error = getError(angle);
//                steer = getSteerDrive(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = -steer;
//                rightSpeed = steer;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.leftFront.setPower(leftSpeed);
//                robot.rightFront.setPower(rightSpeed);
//                robot.leftBack.setPower(leftSpeed);
//                robot.rightBack.setPower(rightSpeed);
//            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            // Turn off RUN_TO_POSITION
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroEncoderStrafe (double speed,
                                   double distance,
                                   double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int leftTarget;
        int rightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftTarget = -(int)(distance * robot.constants.STRAFE_TICKS_PER_IN);
            rightTarget = (int)(distance * robot.constants.STRAFE_TICKS_PER_IN);

            // Set Target and Turn On RUN_TO_POSITION
            setTargetPositionStrafe(leftTarget,rightTarget);

            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFront.isBusy() && robot.rightBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteerDrive(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= 1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFront.setPower(leftSpeed);
                robot.rightFront.setPower(rightSpeed);
                robot.leftBack.setPower(leftSpeed);
                robot.rightBack.setPower(rightSpeed);
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            // Turn off RUN_TO_POSITION
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setIMUOffset() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        sleep(20);

        initialIMUOffset = robot.angles.firstAngle;
    }

    public void setInitLiftPosition() {
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initialLiftPosition = robot.lift.getCurrentPosition();
    }

    public void verifyBackDS(double expectedValue) {
        double currentPosition = robot.distanceBack.getDistance(DistanceUnit.INCH);

        double error = Math.abs(currentPosition - expectedValue);

        if (error > 5) {
            useDistanceSensorBack = false;
        }
    }

    public void alignHorizontally(double targetDistance) {
        DistanceSensor distanceSensor;
        double multiplier = 1;

        if (alliance == AllianceSide.BLUE) {
            distanceSensor = robot.distanceLeftBack;
            multiplier = 1;
        } else {
            distanceSensor = robot.distanceRightBack;
            multiplier = -1;
        }

        double error = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH);
        encoderStrafeBasic(error*multiplier,1,4);


    }


    public void wallAlignFront(double targetDistance, double power, AllianceSide alliance) {
        DistanceSensor distanceSensor = robot.distanceFront;

//        double error = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH);
        if (distanceSensor.getDistance(DistanceUnit.INCH) > targetDistance) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > targetDistance)
                setPower(-power, -power);
        } else if (distanceSensor.getDistance(DistanceUnit.INCH) < targetDistance) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) < targetDistance)
                setPower(power, power);
        }



//        while (Math.abs(error) > 1) {
//            error = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH);
//
//            telemetry.addData("error", error);
//            telemetry.addData("dsval", distanceSensor.getDistance(DistanceUnit.INCH));
//
//            if (error > 0) {
//                strafe(0.85*multiplier);
//            } else {
//                strafe(-0.85*multiplier);
//            }
//        }

        stopMotors();

    }

    public void wallAlignHorizontally(double targetDistance, double power, AllianceSide alliance) {
        DistanceSensor distanceSensor;
        double multiplier = 1;

        if (alliance == AllianceSide.BLUE) {
            distanceSensor = robot.distanceLeftBack;
            multiplier = 1;
        } else {
            distanceSensor = robot.distanceRightBack;
            multiplier = -1;
        }

//        double error = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH);
        if (distanceSensor.getDistance(DistanceUnit.INCH) > targetDistance) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > targetDistance)
                strafe(-power*multiplier);
        } else if (distanceSensor.getDistance(DistanceUnit.INCH) < targetDistance) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) < targetDistance)
                strafe(power*multiplier);
        }



//        while (Math.abs(error) > 1) {
//            error = targetDistance - distanceSensor.getDistance(DistanceUnit.INCH);
//
//            telemetry.addData("error", error);
//            telemetry.addData("dsval", distanceSensor.getDistance(DistanceUnit.INCH));
//
//            if (error > 0) {
//                strafe(0.85*multiplier);
//            } else {
//                strafe(-0.85*multiplier);
//            }
//        }

        stopMotors();

    }

    public void verifyFrontDS(double tgtDist) {
        double error = Math.abs(robot.distanceFront.getDistance(DistanceUnit.INCH) - tgtDist);

        if (error > 21) {
            useDistanceSensorFront = false;
        } else {
            useDistanceSensorFront = true;
        }
    }

    public void moveLift(double targetEnc) {

        robot.lift.setTargetPosition((int)targetEnc);

        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lift.setPower(0.5);

        while (robot.lift.isBusy() && opModeIsActive()) {
            idle();
        }

        robot.lift.setPower(0);
    }

    public void liftTime(double timeS, double power) {
        runtime.reset();

        robot.lift.setPower(-power);

        while(runtime.seconds() < timeS) {
            idle();
        }

        robot.lift.setPower(0);
    }

    public void strafe(double power) {
        robot.leftFront.setPower(-power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(-power);
    }

    public void strafeWallDist(double targetDist, double speed, DistanceSensor inputDistance, Direction direction, double timeoutMS) {

        double currentDist = inputDistance.getDistance(DistanceUnit.INCH);
        double error = targetDist - currentDist;
        // see if we should reverse it so its input-target
        //see if we have actually arrived. Maybe we can run again
        // see if it is too costly to read sensor every time

        if (direction == Direction.RIGHT) {
            while(opModeIsActive() && (Math.abs(error) > 1)) {
                if (error > 0) {
                    strafeGyro(-1, 0);
                    currentDist = inputDistance.getDistance(DistanceUnit.INCH);
                    error = targetDist - currentDist;
//                    telemetry.addData("current dist", inputDistance.getDistance(DistanceUnit.INCH));
//                    telemetry.update();
                }

                else {
                    strafeGyro(1, 0);
                    error = targetDist - inputDistance.getDistance(DistanceUnit.INCH);
//                    telemetry.addData("current dist", inputDistance.getDistance(DistanceUnit.INCH));
//                    telemetry.update();
                }

                error = targetDist - inputDistance.getDistance(DistanceUnit.INCH);
                idle();
            }

            stopMotors();


        } else if (direction == Direction.LEFT) {
            while(opModeIsActive() && (Math.abs(error) > 1.5)
            ) {
                if (error > 0) {
                    strafeGyro(1, 0);
                    error = targetDist - inputDistance.getDistance(DistanceUnit.INCH);
                }

                else {
                    strafeGyro(-1, 0);
                    error = targetDist - inputDistance.getDistance(DistanceUnit.INCH);
                }

                error = targetDist - inputDistance.getDistance(DistanceUnit.INCH);
                idle();
            }


            stopMotors();
        }

        stopMotors();

    }
}
