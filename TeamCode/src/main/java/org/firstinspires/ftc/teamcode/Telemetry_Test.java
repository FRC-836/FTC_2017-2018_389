package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Telemetry Testing", group="Telemetry")
public class Telemetry_Test extends Autonomous_Parent {
    @Override
    public void runAutonomous() {
        telemetry.addAction(backLeftEncoderAction);
        telemetry.addAction(backRightEncoderAction);

        /*
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                telemetry.addData("Back Right Encoder", "%d", backRightDrive.getCurrentPosition());
            }
        });*/

        while (opModeIsActive())
        {
            telemetry.clear();
            telemetry.update();
            if (gamepad1.x)
                telemetry.removeAction(backLeftEncoderAction);
            if (gamepad1.b)
                telemetry.removeAction(backRightEncoderAction);
        }
    }
}