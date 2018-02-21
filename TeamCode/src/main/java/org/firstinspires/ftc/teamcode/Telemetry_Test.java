package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Telemetry Testing", group="Telemetry")
public class Telemetry_Test extends Autonomous_Parent {
    @Override
    public void runAutonomous() {
        telemetry.addAction(backLeftEncoderAction);
        telemetry.addAction(backRightEncoderAction);
        while (opModeIsActive())
        {
            telemetry.update();
        }
    }
}