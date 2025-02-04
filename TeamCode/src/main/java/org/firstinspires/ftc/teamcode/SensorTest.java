package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "function testing", group = "sensor")
public class SensorTest extends Autonomous_Parent{
    @Override
    public void runAutonomous() {
        while (opModeIsActive()) {
            if (gamepad1.dpad_up)
                moveForwardEncoder(1.0);
            if (gamepad1.dpad_down)
                moveBackwardEncoder(1.0);
            if (gamepad1.dpad_left)
                turnLeft(90.0);
            if (gamepad1.dpad_right)
                turnRight(90.0);

            if (gamepad1.y)
                moveForwardEncoder(2.0);
            if (gamepad1.a)
                moveBackwardEncoder(2.0);
            if (gamepad1.x)
                turnLeft(180.0);
            if (gamepad1.b)
                turnRight(180.0);
        }
    }
}
