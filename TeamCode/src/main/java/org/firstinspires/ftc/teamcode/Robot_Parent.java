package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    protected DcMotor spinner = null;
    /*
    protected CRServo forwardRightIntake = null;
    protected CRServo forwardLeftIntake = null;
    protected CRServo backwardRightIntake = null;
    protected CRServo backwardLeftIntake = null;
    */
    protected Servo leftIntake = null;
    protected Servo rightIntake = null;
    protected final double LEFT_CLOSED = 0.5;
    protected final double LEFT_OPEN = 1.0;
    protected final double RIGHT_CLOSED = 0.0;
    protected final double RIGHT_OPEN = 0.5;

    protected PID_Loop liftPID = null;
    protected PID_Loop holdLiftPID = null;
    protected PID_Loop drivePID = null;
    protected PID_Loop turnPID = null;
    protected PID_Loop holdTurnPID = null;


    protected double setpoint = 0;
    protected double error = 0;
    protected double time = 0.0;
    protected double lastError = 0;
    protected double lastTime = 0.0;
    protected final double liftP = 0.001;
    protected final double liftI = 0.0003;
    protected final double liftD = 0.0;
    protected final double holdLiftP = 0.0125;
    protected final double holdLiftI = 0.02;
    protected final double holdLiftD = 0.002;
    protected final double EC_PER_DEGREE_LIFT = 8.475;
    protected final double turnP = 0.0;
    protected final double turnI = 0.0;
    protected final double turnD = 0.0;
    protected final double holdTurnP = 0.0;
    protected final double holdTurnI = 0.0;
    protected final double holdTurnD = 0.0;
    protected final double driveP = 0.0;
    protected final double driveI = 0.0;
    protected final double driveD = 0.0;
    protected final double EC_PER_FT_DRIVE = 1304.8;

    private final double JEWEL_ARM_UP = 0.7;
    private final double JEWEL_ARM_DOWN = 0.2;
    protected final double JEWEL_ARM_FULLY_UP = 1.0; // Servo Position

    @Override
    public void runOpMode() {
        drivePID = new PID_Loop(driveP, driveI, driveD);
        turnPID = new PID_Loop(turnP, turnI, turnD);
        holdTurnPID = new PID_Loop(holdTurnP, holdTurnI, holdTurnD);
        liftPID = new PID_Loop(liftP, liftI, liftD);
        holdLiftPID = new PID_Loop(holdLiftP, holdLiftI, holdLiftD);

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
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        /*
        forwardRightIntake = hardwareMap.get(CRServo.class, "fri");
        forwardLeftIntake =  hardwareMap.get(CRServo.class, "fli");
        backwardLeftIntake = hardwareMap.get(CRServo.class, "bli");
        backwardRightIntake = hardwareMap.get(CRServo.class, "bri");
        */
        // The intakes are switched in their current position.
        leftIntake = hardwareMap.get(Servo.class, "fri");
        rightIntake = hardwareMap.get(Servo.class, "fli");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        jewelArm.setDirection(Servo.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);
        /*
        forwardRightIntake.setDirection(CRServo.Direction.FORWARD);
        backwardLeftIntake.setDirection(CRServo.Direction.FORWARD);
        backwardRightIntake.setDirection(CRServo.Direction.REVERSE);
        forwardLeftIntake.setDirection(CRServo.Direction.REVERSE);
        */
        leftIntake.setDirection(Servo.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.FORWARD);

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    protected void setSpinner(double spinnerPower){
        spinner.setPower(spinnerPower);
    }

    protected void setIntake(double intakePower){
        /*
        forwardLeftIntake.setPower(intakePower);
        forwardRightIntake.setPower(intakePower);
        backwardRightIntake.setPower(intakePower);
        backwardLeftIntake.setPower(intakePower);
        */
    }

    protected void closeIntake()
    {
        leftIntake.setPosition(LEFT_CLOSED);
        rightIntake.setPosition(RIGHT_CLOSED);
    }

    protected void openIntake()
    {
        leftIntake.setPosition(LEFT_OPEN);
        rightIntake.setPosition(RIGHT_OPEN);
    }

    protected void resetArmEncoder()
    {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void resetDriveEncoders()
    {
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double update(double input)
    {
        lastError = error;
        error = setpoint - input;
        lastTime = time;
        time = runtime.seconds();

        // pValue
        pValue = PGAIN * error;

        //iValue
        iValue += IGAIN * (lastError + error) * (0.5) * (time - lastTime);

        //dValue
        dValue = DGAIN * (error - lastError) / (time - lastTime);

        if (isFirstTime)
        {
            iValue = 0.0;
            dValue = 0.0;
            isFirstTime = false;
        }
        return pValue + iValue + dValue;
    }
    public void resetPID(double startingIValue) {
        runtime.reset();
        isFirstTime = true;
        iValue = startingIValue;
    }

}