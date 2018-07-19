package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by user on 7/17/2018.
 */
@Autonomous(name="Autonomous Control Loop Test", group="Main")

public class Autonomous_Control_Loop_Test extends Autonomous_Parent {
    private final double BEEP_EC_PER_DEGREES_180 = 15.8;
    private double heading = calculateHeading();
        @Override
        public void runAutonomous () {
            while (opModeIsActive() &&  (backLeftDrive.getCurrentPosition() < 7356.5)){
                PID_Loop(0.15, 0.10, 0.10);
                setSetpoint(0.0);
                double turnCorrection = update(heading);
                setDrive(1.0, turnCorrection, 0);
            }

        }
    }