package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by rishi on 2019-10-13
 *
 * Robot hardware
 */

public class Robot {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    public HardwareMap ahwmap;

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }

    public void initHw () {
        leftFront = ahwmap.dcMotor.get("lf");
        leftBack = ahwmap.dcMotor.get("lb");
        rightFront = ahwmap.dcMotor.get("rf");
        rightBack = ahwmap.dcMotor.get("rb");
    }

}