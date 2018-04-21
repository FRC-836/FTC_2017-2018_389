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
                moveForwardEncoder(2.25);
                break;
            case UNKNOWN:
            case CENTER:
                moveForwardEncoder(2.875);
                break;
            case RIGHT:
                moveForwardEncoder(3.5);
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);
        turnRight(90.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        setIntake(-0.8, -0.8);
        sleep(STEADY_STATE_SLEEP_TIME);
        // These two steps move the robot from the red platform to the red goal.
        moveStraightTime(0.3, 1000);

        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
        setIntake(0.0, 0.0);
        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        //scoreOneMoreGlyph();
    }
}