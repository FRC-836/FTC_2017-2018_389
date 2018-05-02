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

        knockOffJewel(false);

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(2000);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();
        cryptoboxKey = getPictographKey();

        sleep(STEADY_STATE_SLEEP_TIME);

        moveBackwardEncoder(2.166);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft(90.0);

        displayPicto(cryptoboxKey,1000);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                moveForwardEncoder(0.355);//was orignally 175 then 160
                break;
            case UNKNOWN:
            case CENTER:
                moveForwardEncoder(0.9); // 1.193
                break;
            case RIGHT:
                moveForwardEncoder(1.52); // 1.73
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft(85.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveStraightTime(0.3,500);
        setIntake(-0.8, -0.8);
        moveStraightTime(0.3,500);
        sleep(1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        //scoreOneMoreGlyph();
    }
}