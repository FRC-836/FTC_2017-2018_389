//Wheeled-intake Robot Code
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedReader;

public class Robot_Parent extends LinearOpMode {
    protected DcMotor frontLeftDrive = null;
    protected DcMotor backLeftDrive = null;
    protected DcMotor backRightDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor rightIntake = null;
    protected DcMotor leftIntake = null;
    protected Servo jewelArm = null;
    protected DcMotor flipper = null;

    protected final double JEWEL_ARM_UP = 0.5;
    protected final double JEWEL_ARM_DOWN = 1.0;
    protected final double JEWEL_ARM_FULLY_UP = 0.4; // Servo Position
    protected final int FLIPPED_LOCATION = 150;

    private final boolean SWITCH_INTAKES = false;
    private boolean topOpen = true;
    private boolean bottomOpen = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        jewelArm = hardwareMap.get(Servo.class, "jewel_arm");
        flipper = hardwareMap.get(DcMotor.class, "flipper");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        jewelArm.setDirection(Servo.Direction.FORWARD);
        flipper.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        initializeRobot();

        waitForStart();

        startRobot();
    }

    public void initializeRobot() {

    }

    public void startRobot() {

    }

    protected void setDrive(double leftPower, double rightPower) {
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
    }
    protected void setDrive(double forwardsPower, double turnPower, double strafePower) {
        frontRightDrive.setPower(forwardsPower - turnPower - strafePower);
        backRightDrive.setPower(forwardsPower - turnPower + strafePower);
        frontLeftDrive.setPower(forwardsPower + turnPower + strafePower);
        backLeftDrive.setPower(forwardsPower + turnPower - strafePower);
    }
    protected void setDrive(double frPower, double brPower, double flPower, double blPower) {
        frontRightDrive.setPower(frPower);
        backRightDrive.setPower(brPower);
        frontLeftDrive.setPower(flPower);
        backLeftDrive.setPower(blPower);
    }

    protected void setFlipper(double flipperPower) {
        flipper.setPower(flipperPower);
    }

    protected void setIntake(double rightPos, double leftPos) {
        rightIntake.setPower(rightPos);
        leftIntake.setPower(leftPos);
    }



    protected void intakeOff() {
        //setIntake(0.0);
    }

    protected void setJewelArm(double position) {
        jewelArm.setPosition(position);
    }

    protected void raiseJewelArm() {
        setJewelArm(JEWEL_ARM_UP);
    }

    protected void lowerJewelArm() {
        setJewelArm(JEWEL_ARM_DOWN);
    }


}