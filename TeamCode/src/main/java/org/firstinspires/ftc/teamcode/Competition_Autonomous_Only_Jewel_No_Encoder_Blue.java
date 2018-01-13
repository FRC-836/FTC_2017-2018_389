package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Jewel_Timed Blue Left and Right", group="Backup Jewel")
public class Competition_Autonomous_Only_Jewel_No_Encoder_Blue extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
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
                setDrive(-0.5, 0.5);
                sleep(500);
                setDrive(0.0, 0.0);
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
                setDrive(0.5, -0.5);
                sleep(500);
                setDrive(0.0, 0.0);
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