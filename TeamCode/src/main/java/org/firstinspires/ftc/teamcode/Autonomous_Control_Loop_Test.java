package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by user on 7/17/2018.
 */
@Autonomous(name="Autonomous Control Loop Test", group="Main")

public class Autonomous_Control_Loop_Test extends Autonomous_Parent {

    private final double BEEP_EC_PER_DEGREES_180 = 15.8;

    @Override
    public void runAutonomous () {
        PID_Loop(0.01, 0.0, 0.0);
        setSetpoint(calculateHeading());
        while (opModeIsActive() &&  (backLeftDrive.getCurrentPosition() < 7356.5)){
            double turnCorrection = update(calculateHeading());
            setDrive(1.0, turnCorrection, 0);
        }
    }
}