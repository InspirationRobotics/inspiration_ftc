package org.firstinspires.ftc.teamcode.CV.FOS;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants;

public abstract class GetOrientation {

    Constants constants;
    public DistanceSensor dsLeft;
    public DistanceSensor dsRight;
    public DistanceSensor dsFront;
    public DistanceSensor dsBack;

    public void setDistanceSensors(DistanceSensor distLeft, DistanceSensor distRight, DistanceSensor distFront, DistanceSensor distBack) {
        dsLeft = distLeft;
        dsRight = distRight;
        dsFront = distFront;
        dsBack = distBack;
    }


    public double getXPositionDistanceSensor(double last_position_x) {

        double returnedDistance;

        if (last_position_x >= constants.FIELD_HALFWAY_POINT) {
            double measuredDistance = dsRight.getDistance(DistanceUnit.INCH);
            returnedDistance = (2*constants.FIELD_HALFWAY_POINT) - measuredDistance + constants.RIGHT_DIST_BIAS;
        }

        else {
            double measuredDistance = dsLeft.getDistance(DistanceUnit.INCH);
            returnedDistance = measuredDistance + constants.LEFT_DIST_BIAS;
        }

        return returnedDistance;
    }



    public double getYPositionDistanceSensor(double last_position_y) {

        double returnedDistance;

        if (last_position_y >= constants.FIELD_HALFWAY_POINT) {
            double measuredDistance = dsBack.getDistance(DistanceUnit.INCH);
            returnedDistance = (2*constants.FIELD_HALFWAY_POINT) - measuredDistance + constants.BACK_DIST_BIAS;
        }

        else {
            double measuredDistance = dsFront.getDistance(DistanceUnit.INCH);
            returnedDistance = measuredDistance + constants.FRONT_DIST_BIAS;
        }

        return returnedDistance;
    }

    public double getTargetAngle(double targetX, double targetY, double lp_x, double lp_y) {

        double cp_x = getXPositionDistanceSensor(lp_x);
        double cp_y = getYPositionDistanceSensor(lp_y);

        double diffX = targetX - cp_x;
        double diffY = targetY - cp_y;

        double tgt_angle_rad = Math.atan2((diffY + 0.0000001), (diffX + 0.0000001));
        double tgt_angle = (180*tgt_angle_rad)/(Math.PI);

        if (diffX > 0) {
            if (diffY > 0) {
                tgt_angle = tgt_angle;
            }
            else {
                tgt_angle = tgt_angle + 360;
            }
        }

        else {
            tgt_angle = tgt_angle +180;
        }

        return tgt_angle;
    }

}
