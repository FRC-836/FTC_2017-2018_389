package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

        turnRight_Encoder(25.0);
        sleep(1000);

        moveForwardEncoder(2.0);

        switch (cryptoboxKey) {
            case LEFT:
                turnLeft_Encoder(45.0);
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft_Encoder(30.0);
                break;
            case RIGHT:
                turnLeft_Encoder(20.0);
                break;
        }

        moveStraightTime(0.5, 1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
    }
}
