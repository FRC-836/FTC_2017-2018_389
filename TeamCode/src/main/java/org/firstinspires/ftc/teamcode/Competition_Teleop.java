package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Competition Teleop", group="Competition")
public class Competition_Teleop extends Teleop_Parent
{
    @Override
    public void cycle() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftPower = controllerThreshold(leftPower);
        rightPower = controllerThreshold(rightPower);

        setDrive(leftPower, rightPower);


    }
}