package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Hardware.DistanceSensorType;

/**
 * Created by rishi on 2019-10-13
 *
 * Robot hardware
 */

public class Robot {

    public Constants constants = new Constants();

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor tilter;
    public DcMotor leftCollector;
    public DcMotor rightCollector;
    public Servo autoArm;
    public Servo foundationFront;
    public Servo foundationBack;
    public Servo capStone;

    public DistanceSensor distanceFront;
    public DistanceSensor distanceBack;
    public DistanceSensor distanceFrontLeft;
    public DistanceSensor distanceFrontRight;
    public DistanceSensor distanceBackLeft;
    public DistanceSensor distanceBackRight;
    public DistanceSensor distanceLeft;
    public DistanceSensor distanceRight;

    public DistanceSensor distanceLeftFront;
    public DistanceSensor distanceLeftBack;
    public DistanceSensor distanceRightFront;
    public DistanceSensor distanceRightBack;

    public DcMotor lift;
    public Servo claw;

    public Servo autoCollect;
    public Servo autoPivotLeft;
    public Servo autoPivotRight;
//    public Servo autoExtend;

//    public ModernRoboticsI2cRangeSensor mrDistanceFront;
//    public ModernRoboticsI2cRangeSensor mrDistanceBack;
//    public ModernRoboticsI2cRangeSensor mrDistanceLeft;
//    public ModernRoboticsI2cRangeSensor mrDistanceRight;

    //River
    public DcMotor extension; //for the collector extension
    public DcMotor intake; //for the intake motors inside of the collector
    public DcMotor leftLift; //will actuate the vertical lift (need to be used in tangent with left righLift and vice versa)
    public DcMotor rightLift; //will actuate the vertical lift
    public Servo leftFoundation; // actuate the foundation mover (need to be used in tangent with right foundation and vice versa)
    public Servo rightFoundation; // actuate teh foundation mover
    public Servo leftExtension; //for the grabber extension (need to be used in tangent with the rightExtension
    public Servo rightExtension; //for the grabber extension
    public Servo wrist; // able to be rotated to allow the robot longer reach
    public Servo grabber; //grabbing the nubs on the stone
    public Servo capstone;

    public DistanceSensor intakeDistance;
    public DigitalChannel liftLimit;
    public DigitalChannel extensionLimit;

    public BNO055IMU imu;

    public Orientation angles;
    //public Acceleration gravity;

    public boolean DEBUG = true;

    public HardwareMap ahwmap;

    public BNO055IMU.Parameters parameters;

    //Waterfall
    public Servo frontClawCollect;
    public Servo backClawCollect;
    public Servo backPivot;
    public Servo frontPivot;
    public Servo foundationServo;

    public DcMotor leftIntake;
    public DcMotor rightIntake;

    public DcMotor extensionMotor;

    /*
    These are the devices/hardware that is the same between River and Watefall:

    leftLift; //will actuate the vertical lift (need to be used in tangent with left righLift and vice versa)
    rightLift; //will actuate the vertical lift
    leftFoundation; // actuate the foundation mover (need to be used in tangent with right foundation and vice versa)
    rightFoundation; // actuate teh foundation mover
    */


    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }


    public void initDrivebase() {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void initDistanceSensorsOld(DistanceSensorType sensorType) {
        if (sensorType == DistanceSensorType.REV) {
            distanceFront = ahwmap.get(DistanceSensor.class, constants.FRONT_DISTANCE_SENSOR_NAME);
            distanceBack = ahwmap.get(DistanceSensor.class, constants.BACK_DISTANCE_SENSOR_NAME);
            distanceLeft = ahwmap.get(DistanceSensor.class, constants.LEFT_DISTANCE_SENSOR_NAME);
            distanceRight = ahwmap.get(DistanceSensor.class, constants.RIGHT_DISTANCE_SENSOR_NAME);
        } else {
            distanceFront = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.FRONT_DISTANCE_SENSOR_NAME);
            distanceBack = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.BACK_DISTANCE_SENSOR_NAME);
            distanceLeft = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.LEFT_DISTANCE_SENSOR_NAME);
            distanceRight = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.RIGHT_DISTANCE_SENSOR_NAME);
        }
    }

    public void initRiverSensors() {
        intakeDistance = ahwmap.get(DistanceSensor.class, constants.INTAKE_DISTANCE_SENSOR_NAME);

        liftLimit = ahwmap.get(DigitalChannel.class, constants.LIFT_MAGLIMIT_SENSOR_NAME);
        extensionLimit = ahwmap.get(DigitalChannel.class, constants.EXTENSION_MAGLIMIT_SENSOR_NAME);
    }

    public void initTilter() {
        tilter = ahwmap.dcMotor.get(constants.TILTER_MOTOR_NAME);

        leftCollector = ahwmap.dcMotor.get(constants.LEFT_COLLECTOR_NAME);
        rightCollector = ahwmap.dcMotor.get(constants.RIGHT_COLLECTOR_NAME);

//        leftCollector.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCollector.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initParaguayFoundationMover() {
        autoArm = ahwmap.servo.get(constants.AUTO_ARM_NAME);
        foundationBack = ahwmap.servo.get(constants.FOUNDATION_BACK_NAME);
        foundationFront = ahwmap.servo.get(constants.FOUNDATION_FRONT_NAME);
    }

    public void initRiver(RobotVersion robotVersion) {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        extension = ahwmap.dcMotor.get(constants.EXTENSION_MOTOR_NAME);
        intake = ahwmap.dcMotor.get(constants.INTAKE_MOTOR_NAME);
        leftLift = ahwmap.dcMotor.get(constants.LIFT_LEFT_MOTOR_NAME);
        rightLift = ahwmap.dcMotor.get(constants.RIGHT_LIFT_MOTOR_NAME);
        leftFoundation = ahwmap.servo.get(constants.LEFT_FOUNDAION_SERVO_NAME);
        rightFoundation = ahwmap.servo.get(constants.RIGHT_FOUNDAION_SERVO_NAME);
        leftExtension = ahwmap.servo.get(constants.LEFT_EXTENSION_SERVO_NAME);
        rightExtension = ahwmap.servo.get(constants.RIGHT_EXTENSION_SERVO_NAME);
        wrist = ahwmap.servo.get(constants.WRIST_SERVO_NAME);
        grabber = ahwmap.servo.get(constants.GRABBER_SERVO_NAME);
        capstone = ahwmap.servo.get(constants.CAPSTONE_NAME);

        if (robotVersion == RobotVersion.RIVER) {
            initDistanceSensorsOld(DistanceSensorType.REV);
            initRiverSensors();
        } else {
            //do nothing
        }
    }

    public void initWaterfall() {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake = ahwmap.dcMotor.get("leftIntake");
        rightIntake = ahwmap.dcMotor.get("rightIntake");
        leftLift = ahwmap.dcMotor.get(constants.LIFT_LEFT_MOTOR_NAME);

        foundationServo = ahwmap.servo.get("foundationMover");
        grabber = ahwmap.servo.get(constants.GRABBER_SERVO_NAME);
        frontClawCollect = ahwmap.servo.get(constants.FRONT_CLAW_COLLECT_NAME);
        backClawCollect = ahwmap.servo.get(constants.BACK_CLAW_COLLECT_NAME);
        backPivot = ahwmap.servo.get(constants.FRONT_PIVOT_SERVO_NAME);
        frontPivot = ahwmap.servo.get(constants.BACK_PIVOT_SERVO_NAME);
        extensionMotor = ahwmap.dcMotor.get(constants.EXTENSION_MOTOR_NAME);

        intakeDistance = ahwmap.get(DistanceSensor.class, constants.INTAKE_DISTANCE_SENSOR_NAME);
        liftLimit = ahwmap.get(DigitalChannel.class, constants.LIFT_MAGLIMIT_SENSOR_NAME);
        extensionLimit = ahwmap.get(DigitalChannel.class, constants.EXTENSION_MAGLIMIT_SENSOR_NAME);
        capstone = ahwmap.servo.get(constants.CAPSTONE_NAME);
    }

    public void initDistanceSensors() {
        distanceFrontLeft = ahwmap.get(DistanceSensor.class, constants.FRONT_LEFT_DISTANCE_SENSOR_NAME);
        distanceFrontRight = ahwmap.get(DistanceSensor.class, constants.FRONT_RIGHT_DISTANCE_SENSOR_NAME);
        distanceBackLeft = ahwmap.get(DistanceSensor.class, constants.BACK_LEFT_DISTANCE_SENSOR_NAME);
        distanceBackRight = ahwmap.get(DistanceSensor.class, constants.BACK_RIGHT_DISTANCE_SENSOR_NAME);
        distanceLeft = ahwmap.get(DistanceSensor.class, constants.LEFT_DISTANCE_SENSOR_NAME);
        distanceRight = ahwmap.get(DistanceSensor.class, constants.RIGHT_DISTANCE_SENSOR_NAME);
    }

    public void initStormDrivebase() {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initStormAttachments() {
        lift = ahwmap.dcMotor.get(constants.LIFT_MOTOR_NAME);

        claw = ahwmap.servo.get(constants.CLAW_MOTOR_NAME);

        autoCollect = ahwmap.servo.get(constants.AUTO_COLLECT_SERVO_NAME);
        autoPivotLeft = ahwmap.servo.get(constants.LEFT_AUTO_PIVOT_SERVO_NAME);
        autoPivotRight = ahwmap.servo.get(constants.RIGHT_AUTO_PIVOT_SERVO_NAME);
//        autoExtend = ahwmap.servo.get(constants.AUTO_EXTEND_SERVO_NAME);

        foundationServo = ahwmap.servo.get("foundationServo");
    }

    public void initStormDistanceSensors() {
        distanceLeftFront = ahwmap.get(DistanceSensor.class, "distanceLeftFront");
        distanceRightFront = ahwmap.get(DistanceSensor.class, "distanceRightFront");
        distanceLeftBack = ahwmap.get(DistanceSensor.class, "distanceLeftBack");
        distanceRightBack = ahwmap.get(DistanceSensor.class, "distanceRightBack");
        distanceFront = ahwmap.get(DistanceSensor.class, "distanceFront");
        distanceBack = ahwmap.get(DistanceSensor.class, "distanceBack");
    }



//    public void initIMU() {
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//
//        imu = ahwmap.get(BNO055IMU.class, constants.IMU_NAME);
//        imu.initialize(parameters);
//    }

//    public double getHeading() {
//
//        double heading;
//
//        heading = angles.firstAngle;
//
//        return heading;
//    }

}