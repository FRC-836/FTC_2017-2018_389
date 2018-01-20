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

        //turnRight_Encoder(25.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        moveForwardEncoder(2.0);
        sleep(STEADY_STATE_SLEEP_TIME);

        switch (cryptoboxKey) {
            case LEFT:
                turnLeft_Encoder(75.0);//was originally 60
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft_Encoder(60.0);//was originally 45.0
                break;
            case RIGHT:
                turnLeft_Encoder(35.0);
                break;
        }
        sleep(STEADY_STATE_SLEEP_TIME);

        moveStraightTime(0.5, 1000);
        sleep(STEADY_STATE_SLEEP_TIME);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
    }
}