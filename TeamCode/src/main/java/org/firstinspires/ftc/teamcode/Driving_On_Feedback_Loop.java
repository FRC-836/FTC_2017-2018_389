package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Driving_On_Feedback_Loop", group="Test")
public class Driving_On_Feedback_Loop extends Autonomous_Parent {

    ElapsedTime runtime;

    int input = 0;
    double output = 0.0;
    int setpoint = 0;
    int error = 0;
    double time = 0.0;
    int lastError = 0;
    double lastTime = 0.0;
    double pValue = 0.0;
    double iValue = 0.0;
    double dValue = 0.0;
    double pGain = 0.0;
    double iGain = 0.0;
    double dGain = 0.0;

    double ENCODER_COUNTS_PER_DEGREE = 0.0; // TODO: Find this value (Lift)

    @Override
    public void runAutonomous() {
        runtime = new ElapsedTime();
        liftArmPID(90.0);
    }

    public void liftArmPID(double angles_degrees) {
        runtime.reset();
        resetArmEncoder();
        boolean pidIsRunning = true;
        boolean isFirstTime = true;
        setpoint = (int) Math.round(angles_degrees * ENCODER_COUNTS_PER_DEGREE);
        while (pidIsRunning) {
            input = liftMotor.getCurrentPosition();
            lastError = error;
            error = setpoint - input;
            lastTime = time;
            time = runtime.seconds();

            // pValue
            pValue = pGain * (double) error;

            //iValue
            iValue += iGain * (double) (lastError + error) * (0.5) * (time - lastTime);

            //dValue
            dValue = dGain * (double) (error - lastError) / (time - lastTime);

            if (isFirstTime)
            {
                iValue = 0.0;
                dValue = 0.0;
                isFirstTime = false;
            }
            output = pValue + iValue + dValue;
            setLift(output);
        }
    }

    public void resetDriveEncoders(){
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(250); // TODO: Remove this sleep if possible
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetArmEncoder(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(250); // TODO: Remove this sleep if possible
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}