package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tank Drive", group="Competition")//Competition/Main
public class Competition_Teleop extends Teleop_Parent
{
    private boolean isIdle = true;

    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

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
            //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setLift(LIFT_POWER_UP);
            //isIdle = false;
        }
        else if(gamepad1.left_trigger > 0.1f) {
            //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setLift(LIFT_POWER_DOWN);
            //isIdle = false;
        }
        else {
            /*if (!isIdle) {
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                setLift(LIFT_POWER_IDLE);
                isIdle = true;
            }*/
            setLift(LIFT_POWER_IDLE);
        }

        // Set intake position
        if (gamepad1.right_trigger > 0.1f) {
            closeBothIntakes();
        }
        else if (gamepad1.right_bumper) {
            openBothIntakes();
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
        if (gamepad1.dpad_left && spinner.getCurrentPosition() > 0)
            spinner.setPower(-0.2);
        else if (gamepad1.dpad_right && spinner.getCurrentPosition() < SPUN_LOCATION)
            spinner.setPower(0.2);
        else
            spinner.setPower(0.0);
        /*        if (gamepad1.y) {
            spin();
        }
        if (gamepad1.b)
        {
            openTopIntake();
            spin();
        }*/
        if (gamepad1.a)
        {
            releaseBothIntakes();
        }

        telemetry.addData("Left, Right power","%4.2f, %4.2f", leftPower, rightPower);
        telemetry.addData("Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.addData("Spinner", spinner.getCurrentPosition());
        telemetry.update();
    }
}