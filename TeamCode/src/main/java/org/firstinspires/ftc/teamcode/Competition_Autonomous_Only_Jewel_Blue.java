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

@Autonomous(name="Jewel_Encoder Blue", group="Backup Jewel")
public class Competition_Autonomous_Only_Jewel_Blue extends Autonomous_Parent {

    @Override
    public void startRobot() {
        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();
        sleep(1500);

        // getColorSeen reports what color the BACK ball is
        switch (getColorSeen()) {
            case BLUE:
                // Moving forward knocks off blue.
                telemetry.addLine("Saw blue.");
                telemetry.update();
                //moveForwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnLeft_Encoder(30.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw blue.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                //turnRight_Encoder(30.0);
                break;
            case RED:
                // Moving forward knocks off red.
                telemetry.addLine("Saw red.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnRight_Encoder(30.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw red.");
                telemetry.update();
                //moveForwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                break;
            case NEITHER:
                raiseJewelArm();
                sleep(1000);
                break;
        }
    }
}