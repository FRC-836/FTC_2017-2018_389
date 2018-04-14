//TODO: Test Programs for values.
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
        // TODO: Change Values if needed, just an estimate

        //turnRight(25.0);
        sleep(1000);
        moveBackwardEncoder(3.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnRight(35.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveBackwardEncoder(1.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                turnRight(28.0);// was originally 19, then 28
                break;
            case UNKNOWN:
            case CENTER:
                turnRight(43.0);//was originally 28 then 45
                break;
            case RIGHT:
                turnRight(63.0);//was originally 45
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