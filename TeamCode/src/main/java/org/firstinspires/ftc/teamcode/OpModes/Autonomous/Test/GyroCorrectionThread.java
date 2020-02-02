package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Test;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class GyroCorrectionThread implements Runnable{
    //Odometry wheels
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    public BNO055IMU imu;

    public Orientation angles;

    public double globalOrientationTgt;

    public double P_TURN_COEFF;
    public double CORRECTION_THRESHOLD;

    //Thead run condition
    private boolean isRunning = true;
    public boolean inGyroTurn = false;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime = 5;


    /**
     * This is the constructor for the thread
     * @param leftFront
     * @param leftBack
     * @param rightFront
     * @param rightBack
     * @param imu
     * @param angles
     */
    public GyroCorrectionThread(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, BNO055IMU imu, Orientation angles, double P_TURN_COEFF, double CORRECTION_THRESHOLD) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

        this.imu = imu;
        this.angles = angles;

        this.P_TURN_COEFF = P_TURN_COEFF;
    }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    private double getHeading() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double currentAngle = angles.firstAngle;

        return currentAngle;
    }

    private double getError() {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = globalOrientationTgt - getHeading();
        while (robotError > 180 && isRunning)  robotError -= 360;
        while (robotError <= -180 && isRunning) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        if(error > 0) {
            return Range.clip(error * PCoeff, 0, 1);
        } else {
            return Range.clip(error * PCoeff, -1, 0);
        }
    }

    private void correctOrientation() {

        double originalLFSpeed;
        double originalLBSpeed;
        double originalRFSpeed;
        double originalRBSpeed;

        double error = getError();
        double steer = 0;
        double leftSpeed = 0;
        double rightSpeed = 0;

        if (Math.abs(error) <= CORRECTION_THRESHOLD) {
        }

        else {
            steer = getSteer(error, P_TURN_COEFF);
            rightSpeed  = steer;
            leftSpeed   = -rightSpeed;
        }


    }

    public void changeGyroTurnStatus(boolean revisedStatus) {
        inGyroTurn = revisedStatus;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {

            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
