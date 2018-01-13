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

        telemetry.addLine("Done with Jewel. Looking for Pictograph");
        telemetry.update();

        sleep(2000);

        cryptoboxKey = getPictographKey();

        //turnRight_Encoder(25.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        moveBackwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        turnLeft_Encoder(90.0);
        sleep(STEADY_STATE_SLEEP_TIME);
        moveBackwardEncoder(0.8);
        sleep(STEADY_STATE_SLEEP_TIME);
        switch (cryptoboxKey) {
            case LEFT:
                turnLeft_Encoder(35.0);
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft_Encoder(30.0);
                break;
            case RIGHT:
                turnLeft_Encoder(20.0);
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);

        moveStraightTime(0.5, 1000);
        sleep(1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
    }
}