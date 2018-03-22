package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot_Parent extends LinearOpMode {
    protected DcMotor backLeftDrive = null;
    protected DcMotor backRightDrive = null;
    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor liftMotor = null;
    protected Servo jewelArm = null;

    private final double JEWEL_ARM_UP = 0.7;
    private final double JEWEL_ARM_DOWN = 0.2;
    protected final double JEWEL_ARM_FULLY_UP = 1.0; // Servo Position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        jewelArm = hardwareMap.get(Servo.class, "jewel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        jewelArm.setDirection(Servo.Direction.FORWARD);

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
    }

    protected void setLift(double liftPower) {
        liftMotor.setPower(liftPower);
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