package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Competition Red Left", group="Main")
public class Competition_Autonomous_Red_Left extends Autonomous_Parent {

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

        sleep(1000);
        moveForwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft(30.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveForwardEncoder(1.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                turnRight(79.0);// was originally 2.15
                break;
            case UNKNOWN:
            case CENTER:
                turnRight(88.0);//was originally 2.8
                break;
            case RIGHT:
                turnRight(105.0);//was originally 3.4
                break;
        }

        // These two steps move the robot from the red platform to the red goal.
        moveStraightTime(0.5, 1000);

        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);

        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        scoreOneMoreGlyph();
    }
}