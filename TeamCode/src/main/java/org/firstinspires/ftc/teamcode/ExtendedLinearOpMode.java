package org.firstinspires.ftc.teamcode;
import com.inspiration.inspcv.CameraViewDisplay;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.CV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.Hardware.AllianceSide;
import org.firstinspires.ftc.teamcode.Hardware.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.SkystonePosition;
import org.opencv.core.Mat;


public abstract class ExtendedLinearOpMode extends LinearOpMode {

    public Robot robot = new Robot();
    public SkystoneDetector detector = new SkystoneDetector();
    private ElapsedTime runtime = new ElapsedTime();

    public void initDetector() {
        detector.init(robot.ahwmap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
    }

    public void disableDetector() {
        detector.disable();
    }

    public boolean skyStoneIsVisible(AllianceSide side) {
        boolean visible = false;

        if (detector.isVerifiedSkystone(side)) {
            visible = true;
        }

        else if (!detector.isVerifiedSkystone(side)) {
            visible = false;
        }

        return visible;
    }

    public void setPower (double left_power, double right_power) {

        /** Status: in use
         * Usage: (power for the left side of the drivetrain), (power for the right side of the drivetrain)
         */

        robot.leftFront.setPower(-left_power);
        robot.leftBack.setPower(-left_power);
        robot.rightFront.setPower(-right_power);
        robot.rightBack.setPower(-right_power);
    }

    public void initPuddle(HardwareMap hwmp) {

        /** Status: in use
         * Usage: (hardwareMap passed through to initialize robot in one simple function, rather than calling three seperate ones)
         */

        robot.setHardwareMap(hwmp);
        robot.initParaguayFoundationMover();
        robot.initTilter();
        robot.initDrivebase();
    }

    public void timedDrive(Direction direction, long timeMS, double speed) {

        /** Status: in use
         * Usage: Drive the drive base motors by time with the ability to pick direction and speed
         */

        long startTime = System.currentTimeMillis();
        long endTime = startTime + timeMS;

        while (System.currentTimeMillis() < endTime) {

            if (direction == Direction.LEFT) {
                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(-speed);
            }

            else if (direction == Direction.RIGHT) {
                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.rightBack.setPower(speed);
            }

            else if (direction == Direction.FORWARD) {
                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(speed);
            }

            else if (direction == Direction.BACKWARD) {
                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.rightBack.setPower(-speed);
            }
        }

        stopMotors();

    }

    public void timedMotor(DcMotor inputMotor, long timeMS, double speed) {

        /** Status: in use
         * Usage: Move an input motor for a certain time and speed
         */

        long startTime = System.currentTimeMillis();
        long endTime = startTime + timeMS;

        while (System.currentTimeMillis() < endTime) {
            inputMotor.setPower(speed);
        }

        inputMotor.setPower(0);
    }


    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

    }

    public double getDistance(DistanceSensor inputDistSensor, DistanceUnit distanceUnit) {
        double returnDist = inputDistSensor.getDistance(distanceUnit);

        return returnDist;
    }

    public double getHorizontalDistance() {

        double horoizontalDist;

        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.CM) + robot.constants.LEFT_DIST_BIAS;
        double rightDistance = 358.14 - robot.distanceRight.getDistance(DistanceUnit.CM) - robot.constants.RIGHT_DIST_BIAS;

        if(robot.distanceLeft.getDistance(DistanceUnit.CM) > 245) {
            horoizontalDist = rightDistance;
        }

        else {
            horoizontalDist = leftDistance;
        }

        return horoizontalDist;
    }

    public double getVerticalDistance() {

        double verticalDistance;

        double backDistance = robot.distanceBack.getDistance(DistanceUnit.CM) + robot.constants.BACK_DIST_BIAS;
        double frontDistance = 358.14 - robot.distanceFront.getDistance(DistanceUnit.CM) - robot.constants.FRONT_DIST_BIAS;

        if(robot.distanceFront.getDistance(DistanceUnit.CM) > 245) {
            verticalDistance = backDistance;
        }

        else {
            verticalDistance = frontDistance;
        }

        return verticalDistance;
    }

    public double getTargetAngle(double targetX, double targetY) {

        double targetAngle;
        double cpX = getHorizontalDistance();
        double cpY = getVerticalDistance();

        double diffX = targetX - cpX;
        double diffY = targetY - cpY;

        double targetAngleRad = Math.atan2((diffY + 0.0000001), (diffX + 0.0000001));
        targetAngle = (180*targetAngleRad)/(Math.PI);

        if(diffX < 0 && diffY < 0) {
            targetAngle = targetAngle + 180;
        }
        else if (diffX > 0 && diffY < 0) {
            targetAngle = targetAngle + 180;
        }

        return targetAngle;
    }

    public void strafe(double angle) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double inputAngle = Math.toRadians(angle);

        double powerLeftFront = Range.clip((0.70710678)*(((Math.sin(inputAngle)))+((Math.cos(inputAngle)))), -1, 1);
        double powerLeftBack = Range.clip((0.70710678)*((-(Math.sin(inputAngle)))+(Math.cos(inputAngle))), -1, 1);
        double powerRightFront = Range.clip((0.70710678)*((-Math.sin(inputAngle)))+(Math.cos(inputAngle)), -1, 1);
        double powerRightBack = Range.clip((0.70710678)*((Math.sin(inputAngle)))+(Math.cos(inputAngle)), -1, 1);

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);

    }

    public void strafeNoAngle(double speed) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //speed = -1 is right
        // 1 is left

        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(-speed);
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(speed);

    }

    public double getSkystoneId(){
        double distUnclipped;
        double distClipped;
        distUnclipped = robot.distanceRight.getDistance(DistanceUnit.INCH) + robot.constants.DISTANCE_SENSOR_COLLECTOR_OFFSET;

        if (distUnclipped % 8 <= 4) {
            distClipped = distUnclipped - (distUnclipped % 8);
        } else if (distUnclipped % 8 >= 4) {
            distClipped = distUnclipped + (8 - distUnclipped % 8);
        } else
            distClipped = distUnclipped;

        /* calculate the skystone id by dividing the clipped distance by 8 */
        return distClipped / 8;

    }

    public double getTargetAngle() {
        double targetAngle = getSkystoneId() * 8 - robot.constants.DISTANCE_SENSOR_COLLECTOR_OFFSET;
        return targetAngle;
    }

    public void moveToPos(double targetX, double targetY) {
        double startDiffX = targetX - getHorizontalDistance();
        double startDiffY = targetY - getVerticalDistance();

        double currentDiffX = startDiffX;
        double currentDiffY = startDiffY;

        while(currentDiffX > robot.constants.DIST_SENSOR_THRESHOLD || currentDiffY > robot.constants.DIST_SENSOR_THRESHOLD) {

            currentDiffX = targetX - getHorizontalDistance();
            currentDiffY = targetY - getVerticalDistance();

            double targetAngle = getTargetAngle(targetX, targetY);
            strafe(targetAngle);

            telemetry.addData("X Pos", getHorizontalDistance());
            telemetry.addData("Y Pos", getVerticalDistance());
            telemetry.addData("Diff X", currentDiffX);
            telemetry.addData("Diff Y", currentDiffY);
        }

        stopMotors();
    }

    /*public void wallAlign(double targetDist, Direction direction, DistanceSensor inputDistance, double maintainedAngle, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        if(direction == Direction.FORWARD) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(90, maintainedAngle);
                }

                else {
                    strafeGyro(270, maintainedAngle);
                }

            }
        }



        else if(direction == Direction.BACKWARD) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(270, maintainedAngle);
                }

                else {
                    strafeGyro(90, maintainedAngle);
                }

            }
        }

        else if(direction == Direction.LEFT) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(180, maintainedAngle);
                }

                else {
                    strafeGyro(0, maintainedAngle);
                }

            }
        }

        else if(direction == Direction.RIGHT) {
            while(Math.abs(inputDistance.getDistance(DistanceUnit.INCH) - targetDist) > robot.constants.DISTANCE_THRESHOLD && opModeIsActive() && endTime > System.currentTimeMillis()) {

                double error = inputDistance.getDistance(DistanceUnit.INCH) - targetDist;

                if(error > 0) {
                    strafeGyro(0, maintainedAngle);
                }

                else {
                    strafeGyro(180, maintainedAngle);
                }

            }
        }

        stopMotors();

    }*/

    public void wallAlign(double speed, double distance, DistanceSensor inputDistance, Direction direction, double timeoutMS) {
        runtime.reset();

        while(opModeIsActive() && !onTargetDistance(speed, distance, robot.constants.P_WALL_COEFF, inputDistance, direction) && (runtime.milliseconds() < timeoutMS)){
            telemetry.addData("Distance from wall", inputDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            idle();
            sleep(200); }


        setPower(0, 0);
        telemetry.addLine("Wall Align complete");
        telemetry.update();

    }

    boolean onTargetDistance(double speed, double distance, double PCoeff, DistanceSensor inputDistance, Direction direction){
        double errorDistance;
        double steer;
        boolean onTarget = false;
        double finalSpeed;

        //determine turm power based on error
        errorDistance = getErrorDistance(distance, inputDistance);

        if (Math.abs(errorDistance) <= robot.constants.DISTANCE_THRESHOLD){

            steer = 0.0;
            finalSpeed = 0.0;
            onTarget = true;
        }
        else{

            double steerMultiplier = 1;

            if (direction == Direction.FORWARD) {
                steerMultiplier = 1;
            } else if (direction == Direction.BACKWARD) {
                steerMultiplier = 1;
            }

            steer = getSteerError(errorDistance, PCoeff);
            finalSpeed = speed * steer * steerMultiplier;
        }

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(finalSpeed, finalSpeed);

        telemetry.addData("Target distance","%5.2f",distance);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", errorDistance, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", finalSpeed, finalSpeed);

        return onTarget;
    }
    public double getErrorDistance(double targetDistance, DistanceSensor inputDistance){

        double robotError;
        robotError = inputDistance.getDistance(DistanceUnit.INCH) - targetDistance;

        telemetry.addData("Robot Error","%5.2f",robotError);
        telemetry.update();

        return robotError;

    }

    public double getSteerError(double error , double PCoeff){
        if (error < 0) {
            return Range.clip(error * PCoeff, -1 , -0.15);
        }
        else {
            return Range.clip(error * PCoeff, 0.15 , 1);
        }
    }

    public void gyroTurn(double targetAngle, double speed, double timeoutS) {

        long startTime = System.currentTimeMillis();
        long endTime = (long)(startTime + (1000*timeoutS));

        while (opModeIsActive() && (System.currentTimeMillis() < endTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, targetAngle, robot.constants.P_TURN_COEFF);
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
            rightSpeed  = speed * steer;
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

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
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
            return Range.clip(error * PCoeff, 0.2, 1);
        } else {
            return Range.clip(error * PCoeff, -1, -0.2);
        }
    }

    public void strafeGyro(double speed, double maintainedAngle) {

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double powerLeftBack = 0.8*speed;
        double powerLeftFront = -0.8*speed;
        double powerRightBack = -0.8*speed;
        double powerRightFront = 0.8*speed;

        double error = getError(maintainedAngle);
        double error_proportioned = Range.clip((error*0.1), -0.2, 0.2);

        powerLeftBack = powerLeftBack - error_proportioned;
        powerLeftFront = powerLeftFront - error_proportioned;
        powerRightBack = powerRightBack + error_proportioned;
        powerRightFront = powerRightFront + error_proportioned;

        robot.leftFront.setPower(powerLeftFront);
        robot.leftBack.setPower(powerLeftBack);
        robot.rightFront.setPower(powerRightFront);
        robot.rightBack.setPower(powerRightBack);
    }

    public void encoderDrive(double left_in, double right_in, double speed_l, double speed_r, double timeoutS) {

        int left_distanceEnc = -(int) (robot.constants.COUNTS_PER_INCH * left_in);
        int right_distanceEnc =  -(int) (robot.constants.COUNTS_PER_INCH * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);

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
            sleep(250);

        }

    }

    public void strafeDistSensor(double distance, Direction direction, DistanceSensor inputDistance, double timeoutMS) {

        double lfP = 0;
        double lbP = 0;
        double rfP = 0;
        double rbP = 0;

        if (direction == Direction.LEFT) {
            lfP = -1;
            lbP = 1;
            rfP = 1;
            rbP = -1;
        } else if (direction == Direction.RIGHT) {
            lfP = 1;
            lbP = -1;
            rfP = -1;
            rbP = 1;
        }

        double startTime = System.currentTimeMillis();
        double endTime = startTime + timeoutMS;


        double error = distance - inputDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("Error", error);
        telemetry.update();

        while ((Math.abs(error) > 2) && (System.currentTimeMillis() < endTime) && opModeIsActive()) {

            sleep(10);

            telemetry.addData("Error", error);
            telemetry.update();

            if(error > 0) {
                robot.leftFront.setPower(-lfP);
                robot.leftBack.setPower(-lbP);
                robot.rightFront.setPower(-rfP);
                robot.rightBack.setPower(-rbP);
            } else {
                robot.leftFront.setPower(lfP);
                robot.leftBack.setPower(lbP);
                robot.rightFront.setPower(rfP);
                robot.rightBack.setPower(rbP);
            }

            error = distance - inputDistance.getDistance(DistanceUnit.INCH);

        }

        telemetry.addData("Error", error);
        telemetry.update();
        stopMotors();
    }

    public void setMotorRunMode(DcMotor.RunMode runmode) {
        robot.leftFront.setMode(runmode);
        robot.rightFront.setMode(runmode);
        robot.leftBack.setMode(runmode);
        robot.rightBack.setMode(runmode);
    }

    public void setTargetPosition(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + rightTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);

    }

    public void extend(double speed, double timeS) {
        runtime.reset();

        while(opModeIsActive() && (timeS > runtime.seconds())) {
            robot.extension.setPower(speed);
        }

        robot.extension.setPower(0);
    }

    public void intake(double speed, double timeS, boolean useDistanceSensor) {
        runtime.reset();

        if(!useDistanceSensor) {
            while (opModeIsActive() && (timeS > runtime.seconds())) {
                robot.intake.setPower(speed);
            }
        }

        else {
            while (opModeIsActive() && (timeS > runtime.seconds()) && (robot.intakeDistance.getDistance(DistanceUnit.INCH) > 3)) {
                robot.intake.setPower(speed);
            }
        }
        robot.intake.setPower(0);
    }


    public void lift(double speed, double timeS) {
        runtime.reset();

        while(opModeIsActive() && (timeS > runtime.seconds())) {
            robot.leftLift.setPower(speed);
            robot.rightLift.setPower(speed);
        }

        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);
    }

    public void doEncoderTurn(double speed, int angle) {

        int tgAngle = Math.abs(angle);
        double distance;
        double leftDistance;
        double rightDistance;

        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distance = ((robot.constants.ENCODERS_PER_DEGREE * tgAngle) / robot.constants.COUNTS_PER_INCH);


        telemetry.addLine("Target Positions Calculated");
        telemetry.update();


        if (angle > 0) {
            rightDistance = -distance;
            leftDistance = distance;


            encoderDrive(leftDistance, rightDistance, speed, speed, 5.5);
        } else if (angle < 0) {
            leftDistance = -distance;
            rightDistance = distance;


            encoderDrive(leftDistance, rightDistance, speed, speed, 5.5);
        }

        else {

            encoderDrive(0, 0, speed, speed, 5.5);

        }

    }

    public void initIMU(HardwareMap ahawmp) {

        robot.imu = hardwareMap.get(BNO055IMU.class, robot.constants.IMU_NAME);

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("imu heading", getHeading());

    }

    public SkystonePosition getSkystonePosition(double distanceFromWall, Direction direction) {

        if (direction == Direction.FORWARD) {
            distanceFromWall = distanceFromWall + robot.constants.FRONT_DS_OFFSET_SKYSTONE;
        } else if (direction == Direction.BACKWARD) {
            distanceFromWall = distanceFromWall + robot.constants.BACK_DS_OFFSET_SKYSTONE;
        }

        double l_err = Math.abs(distanceFromWall - robot.constants.LEFT_SKYSTONE_WALL_DIST);
        double r_err = Math.abs(distanceFromWall - robot.constants.RIGHT_SKYSTONE_WALL_DIST);
        double c_err = Math.abs(distanceFromWall - robot.constants.CENTER_SKYSTONE_WALL_DIST);

        SkystonePosition retPos;
        if (l_err <= r_err && l_err <= c_err) {
            retPos = SkystonePosition.LEFT;
        } else if (r_err <= c_err && r_err <= l_err) {
            retPos = SkystonePosition.RIGHT;
        } else {
            retPos = SkystonePosition.CENTER;
        }

        return retPos;
    }

    public void encoderDistDrive(double targetIn, DistanceSensor inputDistance, Direction distanceSensorPositioning, double minDSInput, double maxDSInput, double speed, double timeoutS) {
        double currentDist = inputDistance.getDistance(DistanceUnit.INCH);

    }

    public void encoderStrafe(double units, double speed) {

        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public void encoderStrafeGyro(double units, double speed, double maintainedAngle) {

        int left_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (robot.constants.STRAFE_TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", units);
                telemetry.addData("Target Distance Right (in)", units);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...

                double powerLeftBack = 0.8*speed;
                double powerLeftFront = -0.8*speed;
                double powerRightBack = -0.8*speed;
                double powerRightFront = 0.8*speed;

                double error = getError(maintainedAngle);
                double error_proportioned = Range.clip((error*0.1), -0.2, 0.2);

                powerLeftBack = powerLeftBack - error_proportioned;
                powerLeftFront = powerLeftFront - error_proportioned;
                powerRightBack = powerRightBack + error_proportioned;
                powerRightFront = powerRightFront + error_proportioned;

                robot.leftFront.setPower(Math.abs(powerLeftFront));
                robot.leftBack.setPower(Math.abs(powerLeftBack));
                robot.rightFront.setPower(Math.abs(powerRightFront));
                robot.rightBack.setPower(Math.abs(powerRightBack));
            }
        }
    }

    public void grabBlockFront() {
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_GRAB);
    }

    public void releaseBlockFront() {
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_DOWN);
        robot.frontClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_UP);
    }

    public void retractAutoArmFront() {
        robot.frontPivot.setPosition(robot.constants.FRONT_PIVOT_MID);
    }

    public void grabBlockBack() {
        robot.backClawCollect.setPosition(robot.constants.FRONT_CLAW_COLLECT_MID);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_GRAB);
    }

    public void releaseBlockBack() {
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_DOWN);
        robot.backClawCollect.setPosition(robot.constants.BACK_CLAW_COLLECT_MID);
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_UP);
    }

    public void retractAutoArmBack() {
        robot.backPivot.setPosition(robot.constants.BACK_PIVOT_MID);
    }

    public void setTargetPositionStrafe(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightFront.getCurrentPosition() + leftTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + rightTarget);
        robot.rightFront.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void goToStone(SkystonePosition skystonePosition, String autoSide) {

        double alignTarget = 0;
        DistanceSensor inputDistance =robot.distanceFront;
        Direction direction = Direction.FORWARD;

        if (autoSide == "blue") {

            inputDistance = robot.distanceBack;
            direction = Direction.BACKWARD;

            if (skystonePosition == SkystonePosition.LEFT) {
                alignTarget = robot.constants.CLOSE_WALL_LEFT_SKYSTONE_BLUE_ALIGN_DISTANCE;
            } else if (skystonePosition == SkystonePosition.CENTER) {
                alignTarget = robot.constants.CLOSE_WALL_CENTER_SKYSTONE_BLUE_ALIGN_DISTANCE;
            } else if (skystonePosition == SkystonePosition.RIGHT) {
                alignTarget = robot.constants.CLOSE_WALL_RIGHT_SKYSTONE_BLUE_ALIGN_DISTANCE;
            }
        } else if (autoSide == "red") {

            inputDistance = robot.distanceFront;
            direction = Direction.FORWARD;

            if (skystonePosition == SkystonePosition.LEFT) {
                alignTarget = robot.constants.CLOSE_WALL_LEFT_SKYSTONE_RED_ALIGN_DISTANCE;
            } else if (skystonePosition == SkystonePosition.CENTER) {
                alignTarget = robot.constants.CLOSE_WALL_CENTER_SKYSTONE_RED_ALIGN_DISTANCE;
            } else if (skystonePosition == SkystonePosition.RIGHT) {
                alignTarget = robot.constants.CLOSE_WALL_RIGHT_SKYSTONE_RED_ALIGN_DISTANCE;
            }
        }

        wallAlign(0.7, alignTarget, inputDistance, direction, 6000);

    }

    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return -robot.angles.firstAngle;
    }

}
