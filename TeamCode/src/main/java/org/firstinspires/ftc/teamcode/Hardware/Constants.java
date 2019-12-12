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


    /* --------------------------------------*
    Vision Related Constants
    *----------------------------------------*/




    /* --------------------------------------*
    Robot-Specific  Constants
    *----------------------------------------*/

    public final String LEFT_DISTANCE_SENSOR_NAME = "leftDist";
    public final String RIGHT_DISTANCE_SENSOR_NAME = "rightDist";
    public final String FRONT_DISTANCE_SENSOR_NAME = "frontDist";
    public final String BACK_DISTANCE_SENSOR_NAME = "backDist";

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

    public final String INTAKE_DISTANCE_SENSOR_NAME = "intakeDistance";
    public final String LIFT_MAGLIMIT_SENSOR_NAME = "liftLimit";
    public final String EXTENSION_MAGLIMIT_SENSOR_NAME = "extensionLimit";

    public final double LEFT_FOUNDATION_OPEN_POS = 1;
    public final double LEFT_FOUNDATION_GRAB_POS = 0;
    public final double RIGHT_FOUNDATION_OPEN_POS = 0;
    public final double RIGHT_FOUNDATION_GRAB_POS = 1;

    public final double LEFT_EXTENSION_EXTENDED = 1;
    public final double LEFT_EXTENSION_COMPACTED = 0;
    public final double RIGHT_EXTENSION_EXTENDED = 0;
    public final double RIGHT_EXTENSION_COMPACTED = 1;

    public final double GRABBER_OPEN_POS = 1;
    public final double GRABBER_GRAB_POS = 0;


    // PuddleTeleOp Robot
    public final String TILTER_MOTOR_NAME = "tilter";
    public final String LEFT_COLLECTOR_NAME = "leftCollector";
    public final String RIGHT_COLLECTOR_NAME = "rightCollector";
    public final String AUTO_ARM_NAME = "autoArm";
    public final String FOUNDATION_BACK_NAME = "foundationBack";
    public final String FOUNDATION_FRONT_NAME = "foundationFront";

    /* --------------------------------------*
    Program Specific Constants
    *----------------------------------------*/

    public final double DIST_SENSOR_THRESHOLD = 5;


}
