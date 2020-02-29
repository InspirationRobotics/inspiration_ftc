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
    public final String FRONT_LEFT_DISTANCE_SENSOR_NAME = "distanceFrontLeft";
    public final String BACK_LEFT_DISTANCE_SENSOR_NAME = "distanceBackLeft";
    public final String FRONT_RIGHT_DISTANCE_SENSOR_NAME = "distanceFrontRight";
    public final String BACK_RIGHT_DISTANCE_SENSOR_NAME = "distanceBackRight";

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

    public final String FRONT_CLAW_COLLECT_NAME = "frontClawCollect";
    public final String BACK_CLAW_COLLECT_NAME = "backClawCollect";
    public final String FRONT_PIVOT_SERVO_NAME = "frontPivot";
    public final String BACK_PIVOT_SERVO_NAME = "backPivot";
    public final String EXTENSION_SERVO_NAME = "extension";

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

    public final double GRABBER_GRAB_POS = 0.65;
    public final double GRABBER_OPEN_POS = 0.325;

    public final double INTAKE_MOTOR_INTAKE_SPEED = 1;
    public final double INTAKE_MOTOR_OUTTAKE_SPEED = -1;
    public final double EXTENSION_EXTEND_SPEED = 1;
    public final double EXTENSION_COMPACT_SPEED = -1;

    public final double LIFT_INCREASE_HEIGHT_SPEED = 1;
    public final double LIFT_LOWER_HEIGHT_SPEED = -1;

    public final double COUNTS_PER_MOTOR_REV    = 537.6 ;
    public final double DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
    public final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public final double ENCODER_STRAFE_ERR_THRESHOLD = 15;


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
    public final double P_TURN_COEFF = 0.07;
    public final double P_WALL_COEFF = 0.0375;
    public final double HEADING_THRESHOLD = 1;
    public final double DISTANCE_THRESHOLD = 1;

    public final double STRAFE_CONSTANT = 1.875;
    public final double STRAFE_TICKS_PER_IN = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * STRAFE_CONSTANT) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public final double ROBOT_WIDTH_IN = 15;
    public final double ROBOT_CIRCUMFERENCE = (ROBOT_WIDTH_IN * Math.PI);
    public final double ENCODERS_PER_360 = (ROBOT_CIRCUMFERENCE*(COUNTS_PER_INCH));
    public final double ENCODERS_PER_DEGREE = (ENCODERS_PER_360/360);

    public final double LEFT_SKYSTONE_WALL_DIST = 24;
    public final double CENTER_SKYSTONE_WALL_DIST = 32;
    public final double RIGHT_SKYSTONE_WALL_DIST = 40;

    public final double BACK_DS_OFFSET_SKYSTONE = 6;
    public final double FRONT_DS_OFFSET_SKYSTONE = 0;


    // Waterfall Robot
    public final double FRONT_CLAW_COLLECT_OPEN = 0;
    public final double FRONT_CLAW_COLLECT_MID = 0.2;
    public final double FRONT_CLAW_COLLECT_GRAB = 0.4;
    public final double BACK_CLAW_COLLECT_OPEN = 0.67;
    public final double BACK_CLAW_COLLECT_GRAB = 0.94;
    public final double BACK_CLAW_COLLECT_MID = (BACK_CLAW_COLLECT_OPEN+BACK_CLAW_COLLECT_GRAB)/2;

    public final double FRONT_PIVOT_UP = 0.5;
    public final double FRONT_PIVOT_MID = 0.75;
    public final double FRONT_PIVOT_DOWN = 1;
    public final double BACK_PIVOT_UP = 0;
    public final double BACK_PIVOT_MID = 0.25;
    public final double BACK_PIVOT_DOWN = 1;

    public final double FOUNDATION_SERVO_OPEN_POS = 0;
    public final double FOUNDATION_SERVO_GRAB_POS = 0.6;


    public final double CLOSE_WALL_LEFT_SKYSTONE_BLUE_ALIGN_DISTANCE = 4;
    public final double CLOSE_WALL_CENTER_SKYSTONE_BLUE_ALIGN_DISTANCE = 12;
    public final double CLOSE_WALL_RIGHT_SKYSTONE_BLUE_ALIGN_DISTANCE = 20;
    public final double FAR_WALL_LEFT_SKYSTONE_BLUE_ALIGN_DISTANCE = 28;
    public final double FAR_WALL_CENTER_SKYSTONE_BLUE_ALIGN_DISTANCE = 36;
    public final double FAR_WALL_RIGHT_SKYSTONE_BLUE_ALIGN_DISTANCE = 44;
    public final double CLOSE_WALL_LEFT_SKYSTONE_RED_ALIGN_DISTANCE = 4;
    public final double CLOSE_WALL_CENTER_SKYSTONE_RED_ALIGN_DISTANCE = 12;
    public final double CLOSE_WALL_RIGHT_SKYSTONE_RED_ALIGN_DISTANCE = 20;
    public final double FAR_WALL_LEFT_SKYSTONE_RED_ALIGN_DISTANCE = 28;
    public final double FAR_WALL_CENTER_SKYSTONE_RED_ALIGN_DISTANCE = 36;
    public final double FAR_WALL_RIGHT_SKYSTONE_RED_ALIGN_DISTANCE = 44;


    public final double WALL_DIST_CENTER = 21.75;
    public final double WALL_DIST_STONE = 26.75;
    public final double WALL_DIST_FOUNDATION = 30;


    //Storm Robot
    public final String LIFT_MOTOR_NAME = "lift";
    public final String CLAW_MOTOR_NAME = "claw";

    public final String AUTO_COLLECT_SERVO_NAME = "autoCollect";
    public final String LEFT_AUTO_PIVOT_SERVO_NAME = "autoPivotLeft";
    public final String RIGHT_AUTO_PIVOT_SERVO_NAME = "autoPivotRight";
    public final String AUTO_EXTEND_SERVO_NAME = "autoExtend";

    public final double AUTO_PIVOT_DOWN_POSITION = 0.82;
    public final double AUTO_PIVOT_COMPACT_POSITION = 0.1;
    public final double AUTO_COLLECT_OPEN_POSITION = 1;
    public final double AUTO_COLLECT_GRAB_POSITION = 0.5;
    public final double AUTO_COLLECT_MID_POSITION = 0.85;

    /*CONSTANTS TO DERIVE:
    ENCODER_STRAFE offset (STRAFE CONSTANT)
    Auto Arm Positions:
        Pivot Down
        Pivot Compact
        Claw Collect Open
        Claw Collect Mid (midpoint between open and grab)
        Claw collect Grab
     ROBOT_WIDTH
     WHEEL_CIRCUMFERENCE
     GEAR_RATIO

     */

}
