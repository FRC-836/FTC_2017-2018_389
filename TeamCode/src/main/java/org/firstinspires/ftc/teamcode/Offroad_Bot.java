package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by user on 5/31/2018.
 */
@TeleOp(name = "Offroad_Bot", group = "void")
public class Offroad_Bot extends OpMode{

    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;
    private final double JOYSTICK_THRESHOLD = 0.1;


    private double controllerThreshold(double number){
        if (Math.abs(number) <= JOYSTICK_THRESHOLD) {
            return 0.0;
        }
        else {
            return number;
        }
    }
    private void setDrive(double forwardsPower, double turnPower, double strafePower) {
        right1.setPower(forwardsPower - turnPower - strafePower);
        left1.setPower(forwardsPower + turnPower + strafePower);
        right2.setPower(forwardsPower - turnPower + strafePower);
        left2.setPower(forwardsPower + turnPower - strafePower);
    }

    @Override
    public void init() {
        left1 = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        left1.setDirection(DcMotor.Direction.REVERSE);
        right1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.FORWARD);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        double drive = -controllerThreshold(gamepad1.left_stick_y);
        double strafe  = controllerThreshold(gamepad1.left_stick_x);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);


        // Force robot to only drive straight forward/backward/left1/right
        if (Math.abs(drive) > Math.abs(strafe))
            setDrive(drive, turn, 0.0);
        else
            setDrive(0.0, turn, strafe);

    }
}
