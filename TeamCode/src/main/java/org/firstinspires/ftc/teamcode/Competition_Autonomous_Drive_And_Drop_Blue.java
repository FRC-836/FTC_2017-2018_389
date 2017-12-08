/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Backup Drive - Blue Turn", group="Backup Drive")
public class Competition_Autonomous_Drive_And_Drop_Blue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor liftMotor = null;
    //private CRServo intakeRight = null;
    //private CRServo intakeLeft = null;
    private Servo servoIntake = null;
    private ColorSensor colorSensor = null;
    private Servo jewelArm = null;


    private final double ENCODER_TURN_POWER = 0.7;
    private final double ENCODER_DRIVE_POWER = 0.5;

    private final boolean USE_LEFT_ENCODER = true;

    private final double BEEP_EC_PER_FEET = 1282.0; // Encoder counts per Foot
    private final double BEEP_EC_PER_DEGREES_180 = 21.22;
    private final double BEEP_EC_PER_DEGREES_DEFAULT = BEEP_EC_PER_DEGREES_180;

    private final double DROP_GLYPH_VALUE = 0.1; // Servo Position
    private final double PICK_UP_GLYPH_VALUE = 0.5; // Servo Position
    private final double INTAKE_FULLY_OPEN = 0.0; // Servo Position

    private final double JEWEL_ARM_UP = 0.7; // Servo Position
    private final double JEWEL_ARM_FULLY_UP = 1.0; // Servo Position
    private final double JEWEL_ARM_DOWN = 0.2; // Servo Position

    private final double JEWEL_DRIVE_DISTANCE = 0.4; // feet

    private final double UNCERTAINTY = 0.05; // Amount that (Red/Blue) > 1 or vice-versa to determine a color

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        //intakeRight = hardwareMap.get(CRServo.class, "intake_right");
        //intakeLeft = hardwareMap.get(CRServo.class, "intake_left");
        servoIntake = hardwareMap.get(Servo.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
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

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        startUp();
        moveStraightTime(0.7, 1300);
        sleep(1000);
        setDrive(-0.5, 0.5);
        sleep(1000);
        setDrive(0.0, 0.0);
        sleep(1000);
        moveStraightTime(0.7,2000 );
        dropGlyph();
        moveStraightTime(-0.35, 500);
    }

    private void setDrive(double leftPower, double rightPower) {
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
    }

    private void setLift(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    private void setIntake(double intakePosition) {
        servoIntake.setPosition(intakePosition);
    }

    private void moveStraightTime(double setSpeed, long timeInMilliseconds) {
        setDrive(setSpeed, setSpeed);
        sleep(timeInMilliseconds);
        setDrive(0.0, 0.0);
    }

    /**
    private void moveStraightBackEncoder(double distanceInFeet) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-0.5, -0.5);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive()) ;
        setDrive(0.0, 0.0);
    }
    */



    private void dropGlyph() {
        setIntake(DROP_GLYPH_VALUE);
    }

    private void pickUpGlyph() {
        setIntake(PICK_UP_GLYPH_VALUE);
    }

    private void intakeOff() {
        //setIntake(0.0);
    }

    private void raiseJewelArm() {
        jewelArm.setPosition(JEWEL_ARM_UP);
    }
    private void raiseJewelArmMore() {
        jewelArm.setPosition(JEWEL_ARM_FULLY_UP);
    }
    private void lowerJewelArm() {
        jewelArm.setPosition(JEWEL_ARM_DOWN);
    }



    private void startUp() {
        raiseJewelArmMore(); // Locks jewel arm
        setIntake(INTAKE_FULLY_OPEN);

        waitForStart();
        runtime.reset();

        raiseJewelArm();
        pickUpGlyph();
    }


    }

