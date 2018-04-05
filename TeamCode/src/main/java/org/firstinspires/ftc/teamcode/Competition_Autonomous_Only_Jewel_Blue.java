package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Jewel_Encoder Blue Left and Right", group="Backup Jewel")
public class Competition_Autonomous_Only_Jewel_Blue extends Autonomous_Parent {

    @Override
    public void runAutonomous() {
        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();
        sleep(1500);
        knockOffJewel(true);
    }
}