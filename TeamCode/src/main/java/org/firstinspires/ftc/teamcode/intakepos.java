package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by user on 4/14/2018.
 */
@TeleOp(name = "intake test")
public class intakepos extends Teleop_Parent{
    @Override
    public void cycle() {
        double i1Pos = -gamepad1.left_stick_y;
        double i2Pos = -gamepad1.right_stick_y;
        intake1.setPosition(i1Pos/2.0 + 0.5);
        intake2.setPosition(i2Pos/2.0 + 0.5);
        telemetry.addData("I1","%4.2f",intake1.getPosition());
        telemetry.addData("I2","%4.2f",intake2.getPosition());
        telemetry.update();
    }
}
