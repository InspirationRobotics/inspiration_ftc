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
 * robot hardware as defined by this configuration document:
 * created on 2020-12-20
 */

public class NewRobot
{
    /* Hardware -------------------------------------------------- */
    /* NAME SCHEME
        call them by their config file names, but if it is too vague add the hardware type to the beginning of the object name
     */

    /** REV Expansion Hub */
    /* (drivetrain) motors */
    public DcMotorEx frontLeft = null;        /* p0 */
    public DcMotorEx frontRight = null;       /* p1 */
    public DcMotorEx backLeft = null;         /* p2 */
    public DcMotorEx backRight = null;        /* p3 */

    /* servos */
    public Servo servoWobbleGoal;           /* p5 */

    /* i2c -- distance sensors */
    public DistanceSensor distRBL = null;    /* p0 */
    public DistanceSensor distRBT = null;    /* p1 */
    public DistanceSensor distRFL = null;    /* p2 */
    public DistanceSensor distRFT = null;    /* p3 */


    /* REV Control Hub */
    /* motors */
    public DcMotor wobbleGoal = null;        /* p0 */
    public DcMotorEx shooterOne = null;       /* p1 */
    public DcMotor shooterTwo = null;       /* p2 */
    public DcMotor collector = null;

    /* servos */
    public Servo shooter;                   /* p5 */

    /* i2c -- distance sensors on sides */
    public DistanceSensor distLFL = null;    /* p0 */
    public DistanceSensor distLFT = null;    /* p1 */
    public DistanceSensor distLBL = null;    /* p2 */
    public DistanceSensor distLBT = null;    /* p3 */

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
        distRBL = hwmap.get(DistanceSensor.class, "RBL");
        distRBT = hwmap.get(DistanceSensor.class, "RBT");
        distRFL = hwmap.get(DistanceSensor.class, "RFL");
        distRFL = hwmap.get(DistanceSensor.class, "RFL");

        distLFL = hwmap.get(DistanceSensor.class, "LFL");
        distLFT = hwmap.get(DistanceSensor.class, "LFT");
        distLBL = hwmap.get(DistanceSensor.class, "LBL");
        distLBT = hwmap.get(DistanceSensor.class, "LBT");
    }

    public void initAllServos()
    {
        shooter = hwmap.get(Servo.class, "shooterTilt");
        servoWobbleGoal = hwmap.get(Servo.class, "servoWobbleGoal");
    }

    public void initMiscMotors()
    {
//        collector = hwmap.get(DcMotor.class, "collector");
//        shooterOne = hwmap.get(DcMotorEx.class, "shooterOne");
        wobbleGoal = hwmap.get(DcMotor.class, "wobbleGoal");
    }

    /* Init End ------------------------------------------------- */

}
