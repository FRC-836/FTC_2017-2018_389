package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot_Parent extends LinearOpMode
{
    protected DcMotor backLeftDrive = null;
    protected DcMotor backRightDrive = null;
    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor liftMotor = null;
    protected Servo servoIntake = null;
    protected Servo jewelArm = null;

    private final double DROP_GLYPH_VALUE = 0.1;
    private final double PICK_UP_GLYPH_VALUE = 0.5;

    private final double JEWEL_ARM_UP = 0.7;
    private final double JEWEL_ARM_DOWN = 0.2;

    protected final double LIFT_POWER_UP = 0.39;
    protected final double LIFT_POWER_DOWN = -0.19;
    protected final double LIFT_POWER_IDLE = 0.09;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        //intakeRight = hardwareMap.get(CRServo.class, "intake_right");
        //intakeLeft = hardwareMap.get(CRServo.class, "intake_left");
        servoIntake = hardwareMap.get(Servo.class, "intake");
        jewelArm = hardwareMap.get(Servo.class, "jewel_arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //intakeRight.setDirection(DcMotor.Direction.FORWARD);
        //intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        servoIntake.setDirection(Servo.Direction.FORWARD);
        jewelArm.setDirection(Servo.Direction.FORWARD);

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    protected void setDrive(double leftPower, double rightPower){
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        //long live the balance
    }

    protected void setLift(double liftPower){
        liftMotor.setPower(liftPower);
    }

    protected void setIntake(double intakePosition){
        servoIntake.setPosition(intakePosition);
    }
    protected void dropGlyph() {
        setIntake(DROP_GLYPH_VALUE);
    }
    protected void pickUpGlyph() {
        setIntake(PICK_UP_GLYPH_VALUE);
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