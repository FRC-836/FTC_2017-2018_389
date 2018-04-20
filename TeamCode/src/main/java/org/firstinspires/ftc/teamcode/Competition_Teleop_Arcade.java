package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Arcade Drive", group="Competition")//Competition/Main
public class Competition_Teleop_Arcade extends Teleop_Parent
{
    private boolean isIdle = true;
    private boolean isJewelArmUp = true;
    private boolean isJewelArmReady = true;
    private boolean isSpinnerEnabled = true;

    @Override
    public void cycle() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //leftPower = -controllerThreshold(gamepad1.left_stick_y);
        //rightPower = -controllerThreshold(gamepad1.right_stick_y);

        double drive = -controllerThreshold(gamepad1.left_stick_y);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

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
            openBothIntakes();
        }
        else if (gamepad1.right_bumper) {
            closeBothIntakes();
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
        }
            telemetry.addLine("Mode is Slow");
        if (gamepad1.y) {
            if (isSpinnerEnabled)
                spin();
            isSpinnerEnabled = false;
        } else if (gamepad1.b) {
            if (isSpinnerEnabled) {
                openTopIntake();
                spin();
            }
            isSpinnerEnabled = false;
        } else isSpinnerEnabled = true;
        if (isClockwise && getCWSwitchPressed()){
            setSpinner(SPINNER_SLOW_POWER);
        }
        if (!isClockwise && getCCWSwitchPressed()){
            setSpinner(-SPINNER_SLOW_POWER);
        }
        if (gamepad1.a)
        {
            releaseBothIntakes();
        }

        if (gamepad1.x) {
            if(isJewelArmReady == true)
            {
                if (isJewelArmUp == true) {
                    lowerJewelArm();
                    isJewelArmUp = false;
                } else {
                    raiseJewelArm();
                    isJewelArmUp = true;
                }
                isJewelArmReady = false;
            }

        }
        else{
            isJewelArmReady = true;
        }

        telemetry.addData("Left, Right power","%4.2f, %4.2f", leftPower, rightPower);
        telemetry.addData("Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.addData("cw limit switch", getCWSwitchPressed());
        telemetry.addData("ccw limit switch", getCCWSwitchPressed());
        telemetry.addData("Spinner", spinner.getPower());
        telemetry.addData("I0", intake0.getPosition());
        telemetry.addData("I1", intake1.getPosition());
        telemetry.addData("I2", intake2.getPosition());
        telemetry.addData("I3", intake3.getPosition());
        telemetry.update();
    }


}

/* Teleop Brainstorms: (This is for robot 2!!! IGNORE!!!)
- Wheel intake, 4 wheels, has to roll certain direction to push glyphs to the platform to dump over
- Jewel Arm on right
- Method for dumper (2nd Intake)

- Probably need to lower the power on wheels because it is flying the glyphs off of the dumper platform
- Lock arms that hold intake wheels
 */
