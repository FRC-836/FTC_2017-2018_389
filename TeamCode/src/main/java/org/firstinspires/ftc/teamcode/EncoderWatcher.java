package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EncoderWatcher implements Runnable {

    Telemetry telemetry;
    DcMotor motorToWatch;
    String motorLabel;

    public EncoderWatcher(Telemetry newTelemetry, DcMotor newMotor, String newMotorLabel) {
        telemetry = newTelemetry;
        motorToWatch = newMotor;
        motorLabel = newMotorLabel;
    }

    @Override
    public void run() {
        telemetry.addData(motorLabel, "%d", motorToWatch.getCurrentPosition());
        `
    }
}
