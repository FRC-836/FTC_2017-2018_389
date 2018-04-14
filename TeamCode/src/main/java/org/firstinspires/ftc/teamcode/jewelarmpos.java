package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by user on 4/14/2018.
 */
@TeleOp(name = "jewel test")
public class jewelarmpos extends Teleop_Parent{
    @Override
    public void cycle() {
        double jewelPos = -gamepad1.right_stick_y;
        jewelArm.setPosition(jewelPos/2.0 + 0.5);
        telemetry.addData("Jewel","%4.2f",jewelArm.getPosition());
        telemetry.update();
    }
}
