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

        knockOffJewel(true);

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(2000);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();
        cryptoboxKey = getPictographKey();

        sleep(1000);
        switch (cryptoboxKey) {
            case LEFT:
                moveForwardEncoder(3.5);// was originally 19, then 28
                break;
            case UNKNOWN:
            case CENTER:
                moveForwardEncoder(2.875);//was originally 28 then 45
                break;
            case RIGHT:
                moveForwardEncoder(2.25);//was originally 45
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft(90.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        // These two steps move the robot from the red platform to the red goal.
        moveStraightTime(-0.5, 1000);

        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);

        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        //scoreOneMoreGlyph();
    }
}