package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="LAST RESORT Blue Right Turn", group="Backup Drive")
public class Competition_Autonomous_Drive_And_Drop_Blue extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
        timedLiftUp(SLIGHT_LIFT_TIME);
        moveStraightTime(0.7, 1300);
        sleep(1000);
        setDrive(-0.5, 0.5);
        sleep(1000);
        setDrive(0.0, 0.0);
        sleep(1000);
        moveStraightTime(0.7, 1000);
        scoreGlyph(false);
    }
}