package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Competition/Main", group="Competition")
public class Competition_Teleop extends OpMode
{
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private final double JEWEL_ARM_DOWN = 0.5;
    private final double JEWEL_ARM_UP = 0.0;
    private final double CONTROLLER_THRESHOLD = 0.1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        double drive = controllerThreshold(-gamepad1.left_stick_y);
        double turn  = controllerThreshold(gamepad1.right_stick_x);
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Send calculated power to wheels
        setDrive(leftPower, rightPower);

        // Set lift power
        if (gamepad1.y) {
            setLift(0.5);
        }
        else if(gamepad1.a) {
            setLift(-0.5);
        }
        else {
            setLift(0.0);
        }

        // Set intake power
        if (gamepad1.b) {
            setIntake(0.5);
        }
        else if (gamepad1.x) {
            setIntake(-0.5);
        }
        else {
            setIntake(0.0);
        }
        if(gamepad1.start)
        {
            setJewelArm(JEWEL_ARM_UP);
        }
        else if(gamepad1.back) {
            setJewelArm(JEWEL_ARM_DOWN); }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

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
        //frontLeftDrive.setPower(leftPower);
        //frontRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
    }

    private void setLift(double liftPower){

    }

    private void setIntake(double intakePower){

    }
    private void setJewelArm(double position) {

    }
    private double controllerThreshold(double joystickValue)
    {
        if(joystickValue >= CONTROLLER_THRESHOLD) {
            return joystickValue;
        }
        else if(-joystickValue >= CONTROLLER_THRESHOLD){
            return joystickValue;
    }
        else {
            return 0.0;
        }
    }
}