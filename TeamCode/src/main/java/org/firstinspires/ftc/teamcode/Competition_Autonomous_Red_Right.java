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

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(2000);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();

        cryptoboxKey = getPictographKey();

        //turnRight(25.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        moveForwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        switch (cryptoboxKey) {
            case LEFT:
                turnLeft(52.0);//was originally 45, then 57 then 50
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft(39.0);//was originally 45.0, then 35 then 40
                break;
            case RIGHT:
                turnLeft(29.0);//was orignally 15, then 27 then 28
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