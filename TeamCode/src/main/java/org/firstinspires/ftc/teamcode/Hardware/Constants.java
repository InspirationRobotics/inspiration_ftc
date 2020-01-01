package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by rishi on 2019-10-13
 *
 * A group of common names or values
 *
 * All units are in inches
 */

public class Constants {

    /* --------------------------------------*
    FOS Related Constants
    *----------------------------------------*/

    public final double FIELD_HALFWAY_POINT = 144;

    public final String VUFORIA_KEY = "AffveYv/////AAAAGQ5VbB9zQUgjlHWrneVac2MnNgfMDlq6EwI3tyURgRK6C" +
            "HargOTFidfzKod6GLQwGD4m9MPLkR+0NfUrnY8+o8FqAKcQbrAsjk8ONdkWYTPZDfoBRgDLNWRuB7LU1MOp9KqAWpXB" +
            "JjvH5JCKF/Hxz+beHfVqdWQ0BVZdgGMXG4yEzLN5AI+4NIkQeLvI7Cwz5pIlksoH+rb/e6+YExoWZbQWhDTiRiemlWjvDM" +
            "1z2a0kteGDz0wTyHz48IkV4M0YsSQIFKwu3YB2a1vkB9FiRfMrBI+CyInjgNoO8V0EEOtRc6Vqsf3XbF3fGXricZUhl7RIl5" +
            "M/IkFOgeAZ4ML+JcrjTqfZb2Yh3JNx1me524cK";



    /* --------------------------------------*
    Vision Related Constants
    *----------------------------------------*/




    /*--------------------------------------*
    Robot-Specific  Constants
    *----------------------------------------*/

    public final String LEFT_DISTANCE_SENSOR_NAME = "distanceLeft";
    public final String RIGHT_DISTANCE_SENSOR_NAME = "distanceRight";
    public final String FRONT_DISTANCE_SENSOR_NAME = "distanceFront";
    public final String BACK_DISTANCE_SENSOR_NAME = "distanceBack";

    public final String LEFT_FRONT_MOTOR_NAME = "lf";
    public final String LEFT_BACK_MOTOR_NAME = "lb";
    public final String RIGHT_FRONT_MOTOR_NAME = "rf";
    public final String RIGHT_BACK_MOTOR_NAME = "rb";

    public final double LEFT_DIST_BIAS = 0;
    public final double RIGHT_DIST_BIAS = 0;
    public final double FRONT_DIST_BIAS = 0;
    public final double BACK_DIST_BIAS = 0;

    //River
    public final String EXTENSION_MOTOR_NAME = "extension";
    public final String INTAKE_MOTOR_NAME = "intake";
    public final String LIFT_LEFT_MOTOR_NAME = "leftLift";
    public final String RIGHT_LIFT_MOTOR_NAME = "rightLift";
    public final String LEFT_FOUNDAION_SERVO_NAME = "leftFoundation";
    public final String RIGHT_FOUNDAION_SERVO_NAME = "rightFoundation";
    public final String LEFT_EXTENSION_SERVO_NAME = "leftExtension";
    public final String RIGHT_EXTENSION_SERVO_NAME = "rightExtension";
    public final String WRIST_SERVO_NAME = "wrist";
    public final String GRABBER_SERVO_NAME = "grabber";
    public final String CAPSTONE_NAME = "capstone";

    public final String INTAKE_DISTANCE_SENSOR_NAME = "intakeDistance";
    public final String LIFT_MAGLIMIT_SENSOR_NAME = "liftLimit";
    public final String EXTENSION_MAGLIMIT_SENSOR_NAME = "extensionLimit";

    public final double LEFT_FOUNDATION_OPEN_POS = 1;
    public final double LEFT_FOUNDATION_GRAB_POS = 0;
    public final double RIGHT_FOUNDATION_OPEN_POS = 0;
    public final double RIGHT_FOUNDATION_GRAB_POS = 1;
    public final double WRIST_INSIDE_POS = 0;
    public final double WRIST_SIDE_POS = 1;
    public final double WRIST_OUTSIDE_POS = 1;

    public final double CAPSTONE_UP = 1;
    public final double CAPSTONE_DOWN = 0;

    public final double LEFT_EXTENSION_EXTENDED = 1;
    public final double LEFT_EXTENSION_COMPACTED = 0;
    public final double RIGHT_EXTENSION_EXTENDED = 0;
    public final double RIGHT_EXTENSION_COMPACTED = 1;

    public final double GRABBER_OPEN_POS = 1;
    public final double GRABBER_GRAB_POS = 0;

    public final double INTAKE_MOTOR_INTAKE_SPEED = 1;
    public final double INTAKE_MOTOR_OUTTAKE_SPEED = -1;
    public final double EXTENSION_EXTEND_SPEED = 1;
    public final double EXTENSION_COMPACT_SPEED = -1;

    public final double LIFT_INCREASE_HEIGHT_SPEED = 1;
    public final double LIFT_LOWER_HEIGHT_SPEED = -1;

    public final double COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    public final double DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP
    public final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    public final String IMU_NAME = "imu";

    public final double DISTANCE_SENSOR_COLLECTOR_OFFSET = 9;


    // PuddleTeleOp Robot
    public final String TILTER_MOTOR_NAME = "tilter";
    public final String LEFT_COLLECTOR_NAME = "leftCollector";
    public final String RIGHT_COLLECTOR_NAME = "rightCollector";
    public final String AUTO_ARM_NAME = "autoArm";
    public final String FOUNDATION_BACK_NAME = "foundationBack";
    public final String FOUNDATION_FRONT_NAME = "foundationFront";

    /* --------------------------------------*
    Program Specific Constants
    *---------------------------------------*/

    public final double DIST_SENSOR_THRESHOLD = 5;
    public final double P_TURN_COEFF = 0.1;
    public final double HEADING_THRESHOLD = 1.5;
    public final double DISTANCE_THRESHOLD = 2;

    // Waterfall Robot
    public final double CLAW_COLLECT_OPEN = 1;
    public final double CLAW_COLLECT_CLOSE = 0;

    public final double CLAW_COLLECT_EXTEND = 1;
    public final double CLAW_COLLECT_RETRACT = 0;

}
