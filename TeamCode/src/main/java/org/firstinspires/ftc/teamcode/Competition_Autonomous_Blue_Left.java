package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Competition Blue Left", group="Main")
public class Competition_Autonomous_Blue_Left extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
        relicTrackables.activate();
        telemetry.addLine("Looking for Color");
        telemetry.update();

        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();
        sleep(1500);

        timedLiftUp(SLIGHT_LIFT_TIME);
        knockOffJewel(true);

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(2000);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();
        cryptoboxKey = getPictographKey();

        //turnRight(25.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        moveBackwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft(60.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveForwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                turnLeft(162.0);//was orignally 175 then 160
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft(144.0);//was originally 150 then 140 then 142
                break;
            case RIGHT:
                turnLeft(124.0);//was orignally 100 then 120 then 122
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);

        moveStraightTime(0.5,1000);
        sleep(1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        scoreOneMoreGlyph();
    }
}