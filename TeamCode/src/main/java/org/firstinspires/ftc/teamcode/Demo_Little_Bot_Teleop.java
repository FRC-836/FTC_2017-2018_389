package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Minibot Teleop", group="Demo")

public class Demo_Little_Bot_Teleop extends LinearOpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private final double JOYSTICK_THRESHOLD = 0.1;
    private boolean wereButtonsPressed = false;
    private boolean isArcadeDrive = true;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "l");
        rightMotor = hardwareMap.get(DcMotor.class, "r");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if(isArcadeDrive) {
                double leftStick = -0.75*gamepad1.left_stick_y;
                double rightStick = gamepad1.right_stick_x;

                double leftPower = (controllerThreshold(leftStick + rightStick));
                double rightPower = (controllerThreshold(leftStick - rightStick));

                setDrive(leftPower, rightPower);
            }
            else
            {
                double leftStick = -gamepad1.left_stick_y;
                double rightStick = -gamepad1.right_stick_y;

                double leftPower = 0.75*(controllerThreshold(leftStick));
                double rightPower = 0.75*(controllerThreshold(rightStick));

                setDrive(leftPower, rightPower);
            }

            if (gamepad1.x && gamepad1.y){
                if (!wereButtonsPressed){
                    wereButtonsPressed = true;
                    isArcadeDrive = !isArcadeDrive;
                }
            }
            else
            {
                wereButtonsPressed = false;
            }
            if (isArcadeDrive) {
                telemetry.addLine("Arcade Drive");
            }
            else {
                telemetry.addLine("Tank Drive");
            }
            telemetry.update();
        }
    }
    protected double controllerThreshold(double number) {
        if (Math.abs(number) <= JOYSTICK_THRESHOLD) {
            return 0.0;
        } else {
            return number;
        }
    }
    private void setDrive(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

}
