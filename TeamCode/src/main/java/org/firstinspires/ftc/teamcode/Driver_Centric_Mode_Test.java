package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;

@TeleOp(name="Driver Centric Mode Test", group="Competition")//Competition/Main
public class Driver_Centric_Mode_Test extends Teleop_Parent
{
    private boolean isFastModeButtonEnabled = true;
    private double correctionValueInDegrees;

    @Override
    public void initializeRobot() {
        correctionValueInDegrees = calculateHeading();
    }

    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;


        //leftPower = -controllerThreshold(gamepad1.left_stick_y);
        //rightPower = -controllerThreshold(gamepad1.right_stick_y);

        double away = -controllerThreshold(gamepad1.left_stick_y);
        double side  = controllerThreshold(gamepad1.left_stick_x);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);
        double directionOfJoystick = Math.toDegrees(Math.atan2(side, away));

        double heading = calculateHeading();
        double fieldOrientedHeading = heading - correctionValueInDegrees;
        double robotOrientedCommand = directionOfJoystick - fieldOrientedHeading;

        double inputSize = Math.sqrt(Math.pow(away, 2) + Math.pow(side, 2));
        double strafe = inputSize * Math.sin(Math.toRadians(robotOrientedCommand));
        double forward =  inputSize * Math.cos(Math.toRadians(robotOrientedCommand));

        if (gamepad1.b)
        {
            if (isFastModeButtonEnabled)
            {
                isModeFast = !isModeFast;
                isFastModeButtonEnabled = false;
            }
        }
        else
            isFastModeButtonEnabled = true;

        if (!isModeFast)
        {
            away *= SLOW_DRIVE_SCALE_FACTOR;
            side *= SLOW_DRIVE_SCALE_FACTOR;
            turn *= SLOW_DRIVE_SCALE_FACTOR;

        }

            setDrive(forward, turn, strafe);

        if (gamepad1.right_bumper)
            setIntake(INTAKE_POWER_IN, INTAKE_POWER_IN);
        else if (gamepad1.right_trigger > 0.8f)
            setIntake(INTAKE_POWER_BACKWARDS, INTAKE_POWER_BACKWARDS);
        else if(gamepad1.x)
            setIntake(-INTAKE_FLIP_POWER, INTAKE_FLIP_POWER);
        else if (gamepad1.a)
            setIntake(INTAKE_FLIP_POWER, -INTAKE_FLIP_POWER);
        else
            setIntake(0.0, 0.0);

        if (gamepad1.left_bumper)
            flipper.setPower(-0.35);
        else if (gamepad1.left_trigger > 0.5f)
            flipper.setPower(0.35);
        else
            flipper.setPower(0.0);

        if(gamepad1.dpad_up)
            isModeFast = true;
        else if(gamepad1.dpad_down)
            isModeFast = false;

        if(isModeFast)
            telemetry.addLine("Mode: Fast");
        else
            telemetry.addLine("Mode: Slow");


        telemetry.addData("Left, Right power","%4.2f, %4.2f", away, turn, side);
        telemetry.addData("Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
    }
}