package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Telemetry Testing", group="Telemetry")
public class Telemetry_Test extends Autonomous_Parent {
    @Override
    public void runAutonomous() {
        telemetry.addAction(backLeftEncoderAction);
        telemetry.addAction(backRightEncoderAction);
        /*telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                telemetry.addData("Back Left Encoder", "%d", backLeftDrive.getCurrentPosition());
                telemetry.addData("Back Right Encoder", "%d", backRightDrive.getCurrentPosition());
            }
        });*/

        while (opModeIsActive())
        {
            telemetry.update();
        }
    }
}