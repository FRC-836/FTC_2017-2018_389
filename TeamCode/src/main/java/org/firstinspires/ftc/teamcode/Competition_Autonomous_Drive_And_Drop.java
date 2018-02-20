package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="LAST RESORT Blue Left Red Right", group="Backup Drive")
public class Competition_Autonomous_Drive_And_Drop extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
        timedLiftUp(SLIGHT_LIFT_TIME);
        moveStraightTime(0.5,2000);
        scoreGlyph(false);
    }
}