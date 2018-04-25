package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Arcade Drive", group="Competition")//Competition/Main
public class Competition_Teleop_Arcade extends Teleop_Parent
{
    private boolean isFastModeButtonEnabled = true;

    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //leftPower = -controllerThreshold(gamepad1.left_stick_y);
        //rightPower = -controllerThreshold(gamepad1.right_stick_y);

        double drive = -controllerThreshold(gamepad1.left_stick_y);
        double strafe  = controllerThreshold(gamepad1.left_stick_x);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);

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
            drive *= SLOW_DRIVE_SCALE_FACTOR;
            strafe *= SLOW_DRIVE_SCALE_FACTOR;
            turn *= SLOW_DRIVE_SCALE_FACTOR;

        }

        // Force robot to only drive straight forward/backward/left/right
        if (Math.abs(drive) > Math.abs(strafe))
            setDrive(drive, turn, 0.0);
        else
            setDrive(0.0, turn, strafe);

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


        telemetry.addData("Left, Right power","%4.2f, %4.2f", drive, turn, strafe);
        telemetry.addData("Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
    }
}