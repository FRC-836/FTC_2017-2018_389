package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by user on 7/17/2018.
 */
@Autonomous(name="Autonomous Control Loop Test", group="Main")
public class Autonomous_Control_Loop_Test extends Autonomous_Parent {
    @Override
    public void runAutonomous () {
        PID_Loop(0.01, 0.01, 0.01);
        setSetpoint(calculateHeading());
        while (opModeIsActive() &&  (backLeftDrive.getCurrentPosition() < 7356.5)){
            double turnCorrection = update(calculateHeading());
            setDrive(1.0, -turnCorrection, 0);
        }
        if (opModeIsActive() &&  (backLeftDrive.getCurrentPosition() == 7356)){
            double turnCorrection = 0;

            setDrive(1.0, turnCorrection, 0);
        }
        setDrive(0, 0 , 0);
    }
}