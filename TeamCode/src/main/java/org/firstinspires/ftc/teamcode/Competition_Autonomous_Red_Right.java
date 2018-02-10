package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Competition Red Right", group="Main")
public class Competition_Autonomous_Red_Right extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
        relicTrackables.activate();
        telemetry.addLine("Looking for Color");
        telemetry.update();

        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();
        sleep(1500);

        timedLiftUp(SLIGHT_LIFT_TIME);
        knockOffJewel(false);

        telemetry.addLine("Done with Jewel. Looking for Pictograph");
        telemetry.update();

        sleep(2000);

        cryptoboxKey = getPictographKey();

        //turnRight(25.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        moveForwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        switch (cryptoboxKey) {
            case LEFT:
                turnLeft(45.0);//was originally 60
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft(35.0);//was originally 45.0
                break;
            case RIGHT:
                turnLeft(15.0);
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);

        moveStraightTime(0.5, 1000);
        sleep(STEADY_STATE_SLEEP_TIME);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        scoreOneMoreGlyph();
    }
}