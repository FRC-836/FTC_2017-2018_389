package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Driving_On_Feedback_Loop", group="Test")
public class Driving_On_Feedback_Loop extends LinearOpMode {

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
    double pGain = 0.0007;
    double iGain = 0.00015;
    double dGain = 0.0000000001;

    double ENCODER_COUNTS_PER_DEGREE = 8.475;
    double MULTIPLIER = 1.05;

    DcMotor liftMotor = null;
//1017 encoder counts in 120 degrees
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        liftMotor = hardwareMap.get(DcMotor.class, "arm");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        runtime = new ElapsedTime();
        liftArmPID(0.0);
        liftArmPID(90.0);
        liftArmPID(-90.0);
    }

    public void liftArmPID(double angles_degrees) {
        runtime.reset();
        resetArmEncoder();
        boolean pidIsRunning = true;
        boolean isFirstTime = true;
        setpoint = (int) Math.round(angles_degrees * ENCODER_COUNTS_PER_DEGREE);
        while (pidIsRunning && opModeIsActive()) {
            if (runtime.seconds() > 10.0)
                pidIsRunning = false;
            if (gamepad1.dpad_up)
                pGain *= MULTIPLIER;
            else if (gamepad1.dpad_down)
                pGain /= MULTIPLIER;
            if (gamepad1.y)
                iGain *= MULTIPLIER;
            else if(gamepad1.a)
                iGain /= MULTIPLIER;
            if (gamepad1.right_bumper)
                dGain *= MULTIPLIER;
            else if(gamepad1.right_trigger >= 0.5f)
                dGain /= MULTIPLIER;
            if (gamepad1.left_bumper)
                setpoint += 1;
            else if(gamepad1.left_trigger >= 0.5f)
                setpoint -= 1;
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
            liftMotor.setPower(output);
            telemetry.addData("P","%10.8f", pGain);
            telemetry.addData("I","%10.8f", iGain);
            telemetry.addData("D","%10.8f", dGain);
            telemetry.addData("Set","%d", setpoint);
            telemetry.addData("Loc","%d", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void resetArmEncoder(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(250); // TODO: Remove this sleep if possible
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}