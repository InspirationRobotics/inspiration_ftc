package org.firstinspires.ftc.teamcode.OpModes.Autonomous.LM3;
import org.firstinspires.ftc.teamcode.ExtendedLinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="MoveToSkystone")
public class MoveToSkystone extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {
        robot.setHardwareMap(hardwareMap);
        robot.initWaterfall();
        robot.initDistanceSensors();
        initIMU(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        moveToSkystone(3, 0.125);
        grabAutoArm();

        //collect skystone

        //move to foundation

    }

}