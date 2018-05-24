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

        knockOffJewel(true);

        telemetry.addLine("Done with Jewel. About to look for Pictograph");
        telemetry.update();

        sleep(2000);

        telemetry.addLine("Looking for Pictograph");
        telemetry.update();

        cryptoboxKey = moveForwardEncoderPicto(2.166);
        sleep(500);

        turnLeft(90.0);
        displayPicto(cryptoboxKey,1000);
        sleep(500);
        switch (cryptoboxKey) {
            case LEFT:
                moveForwardEncoder(1.45);//was originally 45, then 57 then 50
                break;
            case UNKNOWN:
            case CENTER:
                moveForwardEncoder(1.11);//was originally 45.0, then 35 then 40
                break;
            case RIGHT:
                moveForwardEncoder(0.385);//was orignally 15, then 27 then 28
                break;
        }
        sleep(500);
        turnRight(90.0);
        sleep(500);
        setIntake(-0.8, -0.8);
        sleep(500);

        moveStraightTime(0.3, 1000);
        sleep(1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);

        if (!RUN_TEST_CODE)
            return;

        //Test code goes beyond this point
        //scoreOneMoreGlyph();*/
    }
}