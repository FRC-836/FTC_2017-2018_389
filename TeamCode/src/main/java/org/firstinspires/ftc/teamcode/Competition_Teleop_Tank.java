package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank Drive", group="Competition")//Competition/Main
public class Competition_Teleop_Tank extends Teleop_Parent
{
    private boolean isFastModeButtonEnabled = true;

    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftDrive = controllerThreshold(-gamepad1.left_stick_y);
        double leftStrafe = controllerThreshold(gamepad1.left_stick_x);
        double rightDrive = controllerThreshold(-gamepad1.right_stick_y);
        double rightStrafe = controllerThreshold(gamepad1.right_stick_x);

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
            leftDrive = leftDrive * SLOW_DRIVE_SCALE_FACTOR;
            leftStrafe = leftStrafe * SLOW_DRIVE_SCALE_FACTOR;
            rightDrive = rightDrive * SLOW_DRIVE_SCALE_FACTOR;
            rightStrafe = rightStrafe * SLOW_DRIVE_SCALE_FACTOR;
        }

        setDrive(rightDrive - rightStrafe,
            rightDrive + rightStrafe,
            leftDrive + leftStrafe,
            leftDrive - leftStrafe);

        if (gamepad1.right_bumper)
            setIntake(INTAKE_POWER_IN, INTAKE_POWER_IN);
        else if (gamepad1.right_trigger > 0.8f)
            setIntake(INTAKE_POWER_BACKWARDS, INTAKE_POWER_BACKWARDS);
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


        telemetry.addData("Left Drive/Strafe","%4.2f / %4.2f", leftDrive, leftStrafe);
        telemetry.addData("Right Drive/Strafe","%4.2f / %4.2f", rightDrive, rightStrafe);
        telemetry.addData("Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
    }
}