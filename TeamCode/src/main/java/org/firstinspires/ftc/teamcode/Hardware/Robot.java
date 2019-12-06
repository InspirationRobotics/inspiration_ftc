package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public DistanceSensor distanceFront;
    public DistanceSensor distanceBack;
    public DistanceSensor distanceLeft;
    public DistanceSensor distanceRight;

    public ModernRoboticsI2cRangeSensor mrDistanceFront;
    public ModernRoboticsI2cRangeSensor mrDistanceBack;
    public ModernRoboticsI2cRangeSensor mrDistanceLeft;
    public ModernRoboticsI2cRangeSensor mrDistanceRight;

    //River
    public DcMotor extension;
    public DcMotor intake;
    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo leftFoundation;
    public Servo rightFoundation;
    public Servo leftExtension;
    public Servo rightExtension;
    public Servo wrist;
    public Servo grabber;

    public HardwareMap ahwmap;

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }


    public void initDrivebase () {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void initDistanceSensors(DistanceSensorType sensorType) {
        if (sensorType == DistanceSensorType.REV) {
            distanceFront = ahwmap.get(DistanceSensor.class, constants.FRONT_DISTANCE_SENSOR_NAME);
            distanceBack = ahwmap.get(DistanceSensor.class, constants.BACK_DISTANCE_SENSOR_NAME);
            distanceLeft = ahwmap.get(DistanceSensor.class, constants.LEFT_DISTANCE_SENSOR_NAME);
            distanceRight = ahwmap.get(DistanceSensor.class, constants.RIGHT_DISTANCE_SENSOR_NAME);
        }

        else {
            mrDistanceFront = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.FRONT_DISTANCE_SENSOR_NAME);
            mrDistanceBack = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.BACK_DISTANCE_SENSOR_NAME);
            mrDistanceLeft = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.LEFT_DISTANCE_SENSOR_NAME);
            mrDistanceRight = ahwmap.get(ModernRoboticsI2cRangeSensor.class, constants.RIGHT_DISTANCE_SENSOR_NAME);
        }
    }

    public void initTilter() {
        tilter = ahwmap.dcMotor.get(constants.TILTER_MOTOR_NAME);

        leftCollector = ahwmap.dcMotor.get(constants.LEFT_COLLECTOR_NAME);
        rightCollector = ahwmap.dcMotor.get(constants.RIGHT_COLLECTOR_NAME);

        rightCollector.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initParaguayFoundationMover() {
        autoArm = ahwmap.servo.get(constants.AUTO_ARM_NAME);
        foundationBack = ahwmap.servo.get(constants.FOUNDATION_BACK_NAME);
        foundationFront = ahwmap.servo.get(constants.FOUNDATION_FRONT_NAME);
    }

    public void initRiver() {
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_MOTOR_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_MOTOR_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_MOTOR_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_MOTOR_NAME);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        extension = ahwmap.dcMotor.get(constants.EXTENSION_MOTOR_NAME);
        intake = ahwmap.dcMotor.get(constants.INTAKE_MOTOR_NAME);
        leftLift = ahwmap.dcMotor.get(constants.EXTENSION_MOTOR_NAME);
        rightLift = ahwmap.dcMotor.get(constants.EXTENSION_MOTOR_NAME);
        leftFoundation = ahwmap.servo.get(constants.LEFT_FOUNDAION_SERVO_NAME);
        rightFoundation = ahwmap.servo.get(constants.RIGHT_FOUNDAION_SERVO_NAME);
        leftExtension = ahwmap.servo.get(constants.LEFT_EXTENSION_SERVO_NAME);
        rightExtension = ahwmap.servo.get(constants.RIGHT_EXTENSION_SERVO_NAME);
        wrist = ahwmap.servo.get(constants.WRIST_SERVO_NAME);
        grabber = ahwmap.servo.get(constants.GRABBER_SERVO_NAME);
    }

}