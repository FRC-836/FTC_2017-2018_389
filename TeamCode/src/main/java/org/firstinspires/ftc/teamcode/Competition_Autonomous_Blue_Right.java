package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Competition Blue Right", group="Main")
public class Competition_Autonomous_Blue_Right extends Autonomous_Parent {

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
        sleep(1000);
        moveBackwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnRight(30.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveBackwardEncoder(1.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                turnRight(19.0);// was originally 2.15
                break;
            case UNKNOWN:
            case CENTER:
                turnRight(28.0);//was originally 2.8
                break;
            case RIGHT:
                turnRight(45.0);//was originally 3.4
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);
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