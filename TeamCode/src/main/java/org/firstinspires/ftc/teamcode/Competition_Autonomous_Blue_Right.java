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

        // getColorSeen reports what color the BACK ball is
        switch (getColorSeen()) {
            case BLUE:
                // Moving forward knocks off blue.
                telemetry.addLine("Saw blue, driving forward.");
                telemetry.update();
                //moveForwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnLeft_Encoder(20.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw blue, driving backward.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                //turnRight_Encoder(30.0);
                break;
            case RED:
                // Moving forward knocks off red.
                telemetry.addLine("Saw red, driving backward.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnRight_Encoder(20.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw red, driving forward.");
                telemetry.update();
                //moveBackwardEncoder();
                turnLeft_Encoder(20.0);

                sleep(1000);
                turnLeft_Encoder(20.0);
                break;
            case NEITHER:
                raiseJewelArm();
                sleep(1000);
                turnLeft_Encoder(20.0);
                break;
        }

        telemetry.addLine("Done with Jewel. Looking for Pictograph");
        telemetry.update();

        sleep(2000);

        cryptoboxKey = getPictographKey();

        turnRight_Encoder(25.0);
        sleep(1000);

        switch (cryptoboxKey) {
            case LEFT:
                moveBackwardEncoder(2.15, ENCODER_DRIVE_POWER);
                break;
            case UNKNOWN:
            case CENTER:
                moveBackwardEncoder(2.8, ENCODER_DRIVE_POWER);
                break;
            case RIGHT:
                moveBackwardEncoder(3.4, ENCODER_DRIVE_POWER);
                break;
        }

        // These two steps move the robot from the red platform to the red goal.
        turnRight_Encoder(90.0);
        moveStraightTime(0.5, 1000);
        // Drops pre-loaded glyph into the cryptobox
        scoreGlyph(true);
    }
}