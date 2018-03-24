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

        if (gamepad1.left_bumper)
            setLift(LIFT_POWER_UP);
        else if(gamepad1.left_trigger > 0.1f)
            setLift(LIFT_POWER_DOWN);
        else
            setLift(0.2);

        if (gamepad1.right_bumper)
            setIntake(PICK_UP_GLYPH_POWER);
        else if (gamepad1.right_trigger > 0.1f)
            setIntake(DROP_GLYPH_POWER);
        else
            setIntake(0.0);

        if (gamepad1.dpad_right)
            setSpinner(0.5);
        else if (gamepad1.dpad_left)
            setSpinner(-0.5);
        else
            setSpinner(0.0);
    }
}