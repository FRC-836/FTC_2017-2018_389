package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Encoder Calibration", group = "sensor")
public class EncoderCalibration extends Autonomous_Parent{
    @Override
    public void runAutonomous() {
        int startPos = backLeftDrive.getCurrentPosition();
        moveStraightTime(ENCODER_DRIVE_POWER,3000);
        while (opModeIsActive())
        {
            telemetry.addData("Relative Encoder Counts","%d",backLeftDrive.getCurrentPosition() - startPos);
            telemetry.addData("Absolute Encoder Counts","%d",backLeftDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
