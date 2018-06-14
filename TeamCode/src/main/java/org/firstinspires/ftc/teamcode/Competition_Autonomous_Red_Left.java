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
        sleep(750);

        knockOffJewel(true);

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(500);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();
        cryptoboxKey = getPictographKey();

        displayPicto(cryptoboxKey,1000);
        switch (cryptoboxKey) {
            case LEFT:
                moveForwardEncoder(3.1);
                break;
            case UNKNOWN:
            case CENTER:
                moveForwardEncoder(2.5165);
                break;
            case RIGHT:
                moveForwardEncoder(1.723);
                break;
        }
        sleep(250);
        turnRight(90.0);
        sleep(250);
        moveStraightTime(0.35, 500);
        setIntake(-0.8, -0.8);
        // These two steps move the robot from the red platform to the red goal.
        moveStraightTime(0.3, 1000);

        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        //scoreOneMoreGlyph();
    }
}