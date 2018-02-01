package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank Drive", group="Competition")//Competition/Main
public class Competition_Teleop extends Teleop_Parent
{
    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        leftPower = -controllerThreshold(gamepad1.left_stick_y);
        rightPower = -controllerThreshold(gamepad1.right_stick_y);

        /*double drive = -controllerThreshold(gamepad1.left_stick_y);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;*/

        if (!isModeFast)
        {
            leftPower *= SLOW_DRIVE_SCALE_FACTOR;
            rightPower *= SLOW_DRIVE_SCALE_FACTOR;
        }
        setDrive(leftPower, rightPower);

        // Set lift power
        if (gamepad1.left_bumper) {
            setLift(LIFT_POWER_UP);
        }
        else if(gamepad1.left_trigger > 0.1f) {
            setLift(LIFT_POWER_DOWN);
        }
        else {
            setLift(LIFT_POWER_IDLE);
        }

        // Set intake position
        if (gamepad1.right_trigger > 0.1f) {
            pickUpGlyph();
        }
        else if (gamepad1.right_bumper) {
            dropGlyph();
        }
        else {
            intakeOff();
        }
        if(gamepad1.dpad_up) {
            isModeFast = true;
        }
        else if(gamepad1.dpad_down) {
            isModeFast = false;
        }
        if(isModeFast) {
            telemetry.addLine("Mode is Fast");
        }
        else {
            telemetry.addLine("Mode is Slow");
        }
        if (gamepad1.a)  {
            double BACK_POWER = -0.15;
            long TIME_ON = 100;
            long TIME_OFF = 100;
            setLift(LIFT_POWER_IDLE);
            setIntake(SLIGHT_INTAKE_VALUE, SLIGHT_INTAKE_VALUE);
            while (gamepad1.a) {
                setDrive(BACK_POWER, BACK_POWER);
                sleep(TIME_ON);
                setDrive(0.0, BACK_POWER);
                sleep(TIME_OFF);
            }
        }
        telemetry.addData("Left, Right power","%4.2f, %4.2f", leftPower, rightPower);
        //telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
    }
}