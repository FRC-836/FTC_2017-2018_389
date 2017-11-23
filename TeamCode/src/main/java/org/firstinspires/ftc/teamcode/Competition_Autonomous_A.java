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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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

@Autonomous(name="Red Left", group="Linear Opmode")
public class Competition_Autonomous_A extends LinearOpMode {

    enum ColorViewed {
        RED,
        BLUE,
        NEITHER
    }

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
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    private final double BEEP_EC_PER_FEET = 1950.0;
    private final double BEEP_EC_PER_DEGREES = 92.2 / 4.0;
    private final double DROP_GLYPH_VALUE = 0.1;
    private final double PICK_UP_GLYPH_VALUE = 0.5;
    private final double INTAKE_FULLY_OPEN = 0.0;
    private final double JEWEL_ARM_UP = 0.7;
    private final double JEWEL_ARM_FULLY_UP = 1.0;
    private final double JEWEL_ARM_DOWN = 0.2;

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
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //intakeRight.setDirection(DcMotor.Direction.FORWARD);
        //intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        servoIntake.setDirection(Servo.Direction.FORWARD);

        setupVuMarkIdentification();

        startUp();

        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();

        // getColorSeen reports what color the BACK ball is
        switch (getColorSeen()) {
            case RED:
                // Moving forward knocks off blue.
                moveStraightRightEncoder(0.4);
                raiseJewelArm();
                moveStraightRightEncoder(-0.4);
                break;
            case BLUE:
                // Moving forward knocks off red.
                moveStraightRightEncoder(-0.4);
                raiseJewelArm();
                moveStraightRightEncoder(0.4);
                break;
            case NEITHER:
                raiseJewelArm();
                break;
        }

        switch (cryptoboxSensor()) {
            case LEFT:
                moveStraightRightEncoder(3.525);
                break;
            case CENTER:
                moveStraightRightEncoder(2.9);
                break;
            case RIGHT:
                moveStraightRightEncoder(2.275);
                break;
        }

        // These two steps move the robot from the red platform to the red goal.
        turnRightEncoder(90);
        moveStraightTime(0.5, 1000);
        // Drops pre-loaded glyph into the cryptobox
        dropGlyph();
        moveStraightRightEncoder(-2.0);
        sleep(1000);

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

    /*private void moveStraightBackEncoder(double distanceInFeet) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-0.5, -0.5);
        while (backRightDrive.getCurrentPosition() < targetPos) ;
        setDrive(0.0, 0.0);
    }*/
    private void moveStraightRightEncoder(double distanceInFeet) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(0.5, 0.5);
        while (backRightDrive.getCurrentPosition() < targetPos) ;
        setDrive(0.0, 0.0);
    }

    private void turnRightEncoder(double degreesOfTurn) {
        int origPos = frontLeftDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * BEEP_EC_PER_DEGREES);
        setDrive(1.0, -1.0);
        while (frontLeftDrive.getCurrentPosition() < targetPos) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, frontLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, frontLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnLeftEncoder(double degreesOfTurn) {
        int origPos = frontLeftDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * BEEP_EC_PER_DEGREES);
        setDrive(-1.0, 1.0);
        while (frontLeftDrive.getCurrentPosition() > targetPos) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, frontLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, frontLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
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

    private void raiseJewelArm() {
        jewelArm.setPosition(JEWEL_ARM_UP);
    }
    private void raiseJewelArmMore() {
        jewelArm.setPosition(JEWEL_ARM_FULLY_UP);
    }
    private void lowerJewelArm() {
        jewelArm.setPosition(JEWEL_ARM_DOWN);
    }

    private ColorViewed getColorSeen() {
        if(colorSensor.blue() == 0){
            return ColorViewed.NEITHER;}
         else if(((double)colorSensor.red()) / ((double)colorSensor.blue()) > 1.2) {
            return ColorViewed.RED;

        } else if (((double)colorSensor.red()) / ((double)colorSensor.blue()) < 0.8) {
            return ColorViewed.BLUE;
        } else {
            return ColorViewed.NEITHER;
        }
    }

    private void startUp() {
        raiseJewelArmMore(); // Locks jewel arm
        setIntake(INTAKE_FULLY_OPEN);

        waitForStart();
        runtime.reset();

        raiseJewelArm();
        pickUpGlyph();
    }

    private RelicRecoveryVuMark cryptoboxSensor() {
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        relicTrackables.deactivate();
        return vuMark;
    }
    private void setupVuMarkIdentification(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Ad/QI4f/////AAAAGTjpPxbdZUqSnVc3mldXKV0E3Ubo8UkPrp0l5P0ie1EXwbAiJNburExxvybAM/e5esxGLn3dl5zN73V9qcvBOROjy68/GQ8c0doo8ApEL127pzLSQEP6rZeq589EtDerLpgCqhsXSnU1hzLJ8S0UcgM9MeeUErzvfso6YAjLGZ9JXzLZHjXlX9lapHT64fBax9lZvMw5pmmZQE/j7oXqeamcdgnKyUn+wQN/3Gb+I2Ye7utY/LFJTiXFYesZsmE/eaq2mGnKVmqA4u6hvrSbEx/QudLqhl3nlXrAUPK+tx+5ersWNIB6OnNaRQApdYJb4mnO8qP8MgWhMIdT4Fuqd3WbitRP1NPUzqO+pJ63c36u";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }
}
