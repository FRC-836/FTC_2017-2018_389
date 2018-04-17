// This is Robot 1 which is the bot that lifts and has grippers as the intake
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.io.BufferedReader;

public class Robot_Parent extends LinearOpMode {
    protected DcMotor backLeftDrive = null;
    protected DcMotor backRightDrive = null;
    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor liftMotor = null;
    protected Servo intake0 = null;
    protected Servo intake1 = null;
    protected Servo intake2 = null;
    protected Servo intake3 = null;
    protected Servo jewelArm = null;
    protected DcMotor spinner = null;
    protected DigitalChannel cwLimitSwitch = null;
    protected DigitalChannel ccwLimitSwitch = null;


    private final double I0_OPEN = 0.9;
    private final double I0_CLOSE = 0.7;
    private final double I1_OPEN = 0.2;
    private final double I1_CLOSE = 0.1;
    private final double I2_OPEN = 0.9;
    private final double I2_CLOSE = 0.7;
    private final double I3_OPEN = 0.9;
    private final double I3_CLOSE = 0.5;
    private final double SLIGHT_INTAKE_OPEN = 0.1;


    private final double JEWEL_ARM_UP = 0.5;
    private final double JEWEL_ARM_DOWN = 0.95;
    protected final double JEWEL_ARM_FULLY_UP = 0.35; // Servo Position

    private final double SPINNER_MAX_POWER = 0.3;
    protected final int SPUN_LOCATION = 720;
    protected boolean isClockwise = true;

    private final boolean SWITCH_INTAKES = true;
    private boolean topOpen = true;
    private boolean bottomOpen = true;

    private final int LIFT_SPIN_LOCATION = 1000;
    protected final double SPINNER_FAST_POWER = 0.75;
    protected final double SPINNER_SLOW_POWER = 0.05;

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
        intake0 = hardwareMap.get(Servo.class, "i0");
        intake1 = hardwareMap.get(Servo.class, "i1");
        intake2 = hardwareMap.get(Servo.class, "i2");
        intake3 = hardwareMap.get(Servo.class, "i3");
        jewelArm = hardwareMap.get(Servo.class, "jewel_arm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        cwLimitSwitch = hardwareMap.get(DigitalChannel.class, "cw");
        ccwLimitSwitch = hardwareMap.get(DigitalChannel.class, "ccw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        intake0.setDirection(Servo.Direction.REVERSE);
        intake1.setDirection(Servo.Direction.REVERSE);
        intake2.setDirection(Servo.Direction.FORWARD);
        intake3.setDirection(Servo.Direction.FORWARD);
        jewelArm.setDirection(Servo.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        /*
        if (isSpinning) {
            if (liftMotor.getCurrentPosition() > LIFT_SPIN_LOCATION) {
                if (liftMotor.getTargetPosition() < LIFT_SPIN_LOCATION)
                    liftMotor.setTargetPosition(LIFT_SPIN_LOCATION);
                finishSpin();
            }
        } else {
            liftMotor.setPower(liftPower);
        }*/
        liftMotor.setPower(liftPower);
    }

    protected void setIntake(double i0Pos, double i1Pos, double i2Pos, double i3Pos) {
        intake0.setPosition(i0Pos);
        intake1.setPosition(i1Pos);
        intake2.setPosition(i2Pos);
        intake3.setPosition(i3Pos);
    }

    protected void openBothIntakes() {
        setIntake(I0_OPEN, I1_OPEN, I2_OPEN, I3_OPEN);
    }

    protected void closeBothIntakes() {
        setIntake(I0_CLOSE, I1_CLOSE, I2_CLOSE, I3_CLOSE);
    }

    protected void releaseBothIntakes() {
        setIntake(I0_CLOSE + SLIGHT_INTAKE_OPEN, I1_CLOSE + SLIGHT_INTAKE_OPEN,
                I2_CLOSE + SLIGHT_INTAKE_OPEN, I3_CLOSE + SLIGHT_INTAKE_OPEN);
    }

    protected void openBottomIntake() {
        if (isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_OPEN, I1_OPEN, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_OPEN, I3_OPEN);
        }
    }

    protected void closeBottomIntake() {
        if (isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_CLOSE, I1_CLOSE, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_CLOSE, I3_CLOSE);
        }
    }

    protected void releaseBottomIntake() {
        if (isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_CLOSE + SLIGHT_INTAKE_OPEN, I1_CLOSE + SLIGHT_INTAKE_OPEN, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_CLOSE + SLIGHT_INTAKE_OPEN, I3_CLOSE + SLIGHT_INTAKE_OPEN);
        }
    }

    protected void openTopIntake() {
        if (!isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_OPEN, I1_OPEN, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_OPEN, I3_OPEN);
        }
    }

    protected void closeTopIntake() {
        if (!isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_CLOSE, I1_CLOSE, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_CLOSE, I3_CLOSE);
        }
    }

    protected void releaseTopIntake() {
        if (!isClockwise ^ SWITCH_INTAKES) {
            setIntake(I0_CLOSE + SLIGHT_INTAKE_OPEN, I1_CLOSE + SLIGHT_INTAKE_OPEN, intake2.getPosition(), intake3.getPosition());
        } else {
            setIntake(intake0.getPosition(), intake1.getPosition(), I2_CLOSE + SLIGHT_INTAKE_OPEN, I3_CLOSE + SLIGHT_INTAKE_OPEN);
        }
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

    protected void setSpinner(double spinnerPower) {
        spinner.setPower(spinnerPower);
    }

    protected void lowerJewelArm() {
        setJewelArm(JEWEL_ARM_DOWN);
    }

    protected void spin() {
        if (isClockwise == true) {
            setSpinner(-SPINNER_FAST_POWER);
            isClockwise = false;
        } else {
            setSpinner(SPINNER_FAST_POWER);
            isClockwise = true;
        }
    }
}
