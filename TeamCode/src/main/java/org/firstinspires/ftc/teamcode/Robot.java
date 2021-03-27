package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * robot hardware as defined by this configuration document: https://docs.google.com/document/d/1KFLzqu7Pr19PzTmmKGaU6Zq64Gp2DHFgC8-qw7VF-Us/edit
 * created on 2020-12-20
 */

public class Robot
{
    /* Hardware -------------------------------------------------- */
    /* NAME SCHEME
        call them by their config file names, but if it is too vague add the hardware type to the beginning of the object name
     */

    /** REV Control Hub */
    /* (drivetrain) motors */
    public DcMotorEx frontLeft = null;        /* p0 */
    public DcMotorEx frontRight = null;       /* p1 */
    public DcMotorEx backLeft = null;         /* p2 */
    public DcMotorEx backRight = null;        /* p3 */

    /* servos */
    public Servo shooter;                   /* p0 */
    public Servo servoWobbleGoal;           /* p1 */

    /* i2c -- distance sensors */
    public DistanceSensor distFL = null;    /* p0 */
    public DistanceSensor distFR = null;    /* p1 */
    public DistanceSensor distBL = null;    /* p2 */
    public DistanceSensor distBR = null;    /* p3 */


    /* REV Expansion Hub */
    /* motors */
    public DcMotor collector = null;        /* p0 */
    public DcMotorEx shooterOne = null;       /* p1 */
    public DcMotor shooterTwo = null;       /* p2 */
    public DcMotor wobbleGoal = null;

    /* i2c -- distance sensors on sides */
    public DistanceSensor distLF = null;    /* p0 */
    public DistanceSensor distRF = null;    /* p1 */
    public DistanceSensor distLB = null;    /* p2 */
    public DistanceSensor distRB = null;    /* p3 */

    /* imu */
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;


    /* HW End --------------------------------------------------- */

    /* Init Procedures ------------------------------------------ */

    /* local OpMode members. */
    public HardwareMap hwmap           =  null;

    /* call this before hw initialization */
    public void setHardwareMap(HardwareMap ahwmap) { hwmap = ahwmap; }

    public void initDrivetrain()
    {
        frontLeft = hwmap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwmap.get(DcMotorEx.class, "frontRight");
        backLeft = hwmap.get(DcMotorEx.class, "backLeft");
        backRight = hwmap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* if you want to init them all at once use this */
    public void initAllDistanceSensors()
    {
        distFL = hwmap.get(DistanceSensor.class, "FL");
        distFR = hwmap.get(DistanceSensor.class, "FR");
        distBL = hwmap.get(DistanceSensor.class, "BL");
        distBR = hwmap.get(DistanceSensor.class, "BR");

        distLF = hwmap.get(DistanceSensor.class, "LF");
        distRF = hwmap.get(DistanceSensor.class, "RF");
        distLB = hwmap.get(DistanceSensor.class, "LB");
        distRB = hwmap.get(DistanceSensor.class, "RB");
    }

    public void initAllServos()
    {
        shooter = hwmap.get(Servo.class, "shooter");
        servoWobbleGoal = hwmap.get(Servo.class, "servoWobbleGoal");
    }

    public void initMiscMotors()
    {
        collector = hwmap.get(DcMotor.class, "collector");
        shooterOne = hwmap.get(DcMotorEx.class, "shooterOne");
        // shooterTwo = hwmap.get(DcMotor.class, "shooterTwo");
        wobbleGoal = hwmap.get(DcMotor.class, "wobbleGoal");
    }

    public void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    
    /* Init End ------------------------------------------------- */

}
