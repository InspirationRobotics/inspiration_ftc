package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FOS.DistanceSensorType;

/**
 * Created by rishi on 2019-10-13
 *
 * Robot hardware
 */

public class Robot {

    Constants constants;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DistanceSensor distanceFront;
    DistanceSensor distanceBack;
    DistanceSensor distanceLeft;
    DistanceSensor distanceRight;

    ModernRoboticsI2cRangeSensor mrDistanceFront;
    ModernRoboticsI2cRangeSensor mrDistanceBack;
    ModernRoboticsI2cRangeSensor mrDistanceLeft;
    ModernRoboticsI2cRangeSensor mrDistanceRight;

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

}