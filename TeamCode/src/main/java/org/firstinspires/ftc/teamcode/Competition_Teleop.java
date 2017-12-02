package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank Drive", group="Competition")//Competition/Main
public class Competition_Teleop extends OpMode
{
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor liftMotor = null;
    private Servo servoIntake = null;
    private Servo jewelArm = null;

    private boolean isModeFast = true;

    private final double DROP_GLYPH_VALUE = 0.1;
    private final double PICK_UP_GLYPH_VALUE = 0.5;

    private final double JEWEL_ARM_UP = 0.7;
    private final double JEWEL_ARM_DOWN = 0.2;

    private final double LIFT_POWER_UP = 0.39;
    private final double LIFT_POWER_DOWN = -0.19;
    private final double LIFT_POWER_IDLE = 0.09;

    private final double JOYSTICK_THRESHOLD = 0.1;
    private final double SLOW_DRIVE_SCALE_FACTOR = 0.5;

    @Override
    public void init() {
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
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        leftPower = -controllerThreshold(gamepad1.left_stick_y);
        rightPower = -controllerThreshold(gamepad1.right_stick_y);

        /*double drive = -controllerThreshold(gamepad1.left_stick_y);
        double turn  =  controllerThreshold(gamepad1.right_stick_x);
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;*/

        // Send calculated power to wheels
        if(isModeFast) {
            setDrive(leftPower, rightPower);
        }
        else {
            setDrive(leftPower * SLOW_DRIVE_SCALE_FACTOR, rightPower * SLOW_DRIVE_SCALE_FACTOR);
        }
        // Set lift power
        if (gamepad1.left_bumper) {
            setLift(LIFT_POWER_UP);
        }
        else if(gamepad1.left_trigger > 0.1f) {
            setLift(LIFT_POWER_DOWN);
        }
        else {
            setLift(LIFT_POWER_IDLE);
        }

        // Set intake position
        if (gamepad1.right_trigger > 0.1f) {
            pickUpGlyph();
        }
        else if (gamepad1.right_bumper) {
            dropGlyph();
        }
        else {
            intakeOff();
        }

        // Set jewel arm
        if(gamepad1.y) {
            raiseJewelArm();
        }
        else if(gamepad1.x) {
            lowerJewelArm();
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
        else{
            telemetry.addLine("Mode is Slow");
        }
        //telemetry.addData("Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void setDrive(double leftPower, double rightPower){
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        //long live the balance
    }

    private void setLift(double liftPower){
        liftMotor.setPower(liftPower);
    }

    private void setIntake(double intakePosition){
        servoIntake.setPosition(intakePosition);
    }
    private void dropGlyph() {
        setIntake(DROP_GLYPH_VALUE);
    }
    private void pickUpGlyph() {
        setIntake(PICK_UP_GLYPH_VALUE);
    }
    private void intakeOff() {
        //setIntake(0.0);
    }

    private void setJewelArm(double position) {
        jewelArm.setPosition(position);
    }
    private void raiseJewelArm() {
        setJewelArm(JEWEL_ARM_UP);
    }
    private void lowerJewelArm() {
        setJewelArm(JEWEL_ARM_DOWN);
    }

    private double controllerThreshold(double number){
        if (Math.abs(number) <= JOYSTICK_THRESHOLD) {
            return 0.0;
        }
        else {
            return number;
        }
    }
}