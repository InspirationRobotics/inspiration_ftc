package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public DcMotor frontLeft = null;        /* p0 */
    public DcMotor frontRight = null;       /* p1 */
    public DcMotor backLeft = null;         /* p2 */
    public DcMotor backRight = null;        /* p3 */

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
    public DcMotor shooterOne = null;       /* p1 */
    public DcMotor shooterTwo = null;       /* p2 */
    public DcMotor motorWobbleGoal = null;

    /* i2c -- distance sensors on sides */
    public DistanceSensor distLF = null;    /* p0 */
    public DistanceSensor distRF = null;    /* p1 */
    public DistanceSensor distLB = null;    /* p2 */
    public DistanceSensor distRB = null;    /* p3 */


    /* HW End --------------------------------------------------- */

    /* Init Procedures ------------------------------------------ */

    /* local OpMode members. */
    HardwareMap hwmap           =  null;

    /* call this before hw initialization */
    public void setHardwareMap(HardwareMap ahwmap) { hwmap = ahwmap; }

    public void initDrivetrain()
    {
        frontLeft = hwmap.get(DcMotor.class, "frontLeft");
        frontRight = hwmap.get(DcMotor.class, "frontRight");
        backLeft = hwmap.get(DcMotor.class, "backLeft");
        backRight = hwmap.get(DcMotor.class, "backRight");
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
        shooterOne = hwmap.get(DcMotor.class, "shooterOne");
        shooterTwo = hwmap.get(DcMotor.class, "shooterTwo");
        motorWobbleGoal = hwmap.get(DcMotor.class, "motorWobbleGoal");
    }

    /* Init End ------------------------------------------------- */

}