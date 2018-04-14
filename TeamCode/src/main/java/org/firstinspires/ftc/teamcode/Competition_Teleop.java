package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank Drive", group="Competition")//Competition/Main
public class Competition_Teleop extends Teleop_Parent
{
    private boolean isIdle = true;
    private boolean isJewelArmUp = true;
    private boolean isJewelArmReady = true;


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
        if (gamepad1.y) {
            spin();
        }
        if (gamepad1.b)
        {
            openTopIntake();
            spin();
        }
        if (isClockwise && cwLimitSwitch.getState()){
            setSpinner(SPINNER_SLOW_POWER);
        }
        if (!isClockwise && ccwLimitSwitch.getState()){
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
        telemetry.addData("Spinner", spinner.getCurrentPosition());
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
