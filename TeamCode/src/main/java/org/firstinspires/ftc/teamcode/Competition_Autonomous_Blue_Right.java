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

        telemetry.addLine("Done with Jewel. Looking for Pictograph");
        telemetry.update();

        sleep(2000);

        cryptoboxKey = getPictographKey();

        //turnRight_Encoder(25.0);
        sleep(1000);

        switch (cryptoboxKey) {
            case LEFT:
                moveBackwardEncoder(1.6, ENCODER_DRIVE_POWER);// was originally 2.15
                break;
            case UNKNOWN:
            case CENTER:
                moveBackwardEncoder(2.15, ENCODER_DRIVE_POWER);//was originally 2.8
                break;
            case RIGHT:
                moveBackwardEncoder(2.8, ENCODER_DRIVE_POWER);//was originally 3.4
                break;
        }

        // These two steps move the robot from the red platform to the red goal.
        turnRight_Encoder(90.0);
        moveStraightTime(0.5, 1000);

        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);

        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        scoreOneMoreGlyph();
    }
}