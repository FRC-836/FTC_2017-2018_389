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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

@Autonomous(name="Blue Left", group="Main")
public class Competition_Autonomous_Blue_Left extends LinearOpMode {

    //Vuforia Variables
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters vParameters;
    VuforiaTrackables relicTrackables;
    private RelicRecoveryVuMark cryptoboxKey = null;
    private VuforiaTrackable relicTemplate = null;
    VuforiaLocalizer vuforia;

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

        setupVuMarkData();

        startUp();
        relicTrackables.activate();
        telemetry.addLine("Looking for Color");
        telemetry.update();

        //Move Jewel arm to where it sees a jewel
        lowerJewelArm();
        sleep(1500);

        // getColorSeen reports what color the BACK ball is
        switch (getColorSeen()) {
            case BLUE:
                // Moving forward knocks off blue.
                telemetry.addLine("Saw blue, driving forward.");
                telemetry.update();
                //moveForwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnLeft_Encoder(30.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw blue, driving backward.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                //turnRight_Encoder(30.0);
                break;
            case RED:
                // Moving forward knocks off red.
                telemetry.addLine("Saw red, driving backward.");
                telemetry.update();
                //moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnRight_Encoder(30.0);
                raiseJewelArm();
                sleep(1000);
                telemetry.addLine("Saw red, driving forward.");
                telemetry.update();
                //moveForwardEncoder(JEWEL_DRIVE_DISTANCE, ENCODER_DRIVE_POWER);
                turnLeft_Encoder(30.0);

                sleep(1000);
                turnLeft_Encoder(30.0);
                break;
            case NEITHER:
                raiseJewelArm();
                sleep(1000);
                turnLeft_Encoder(30.0);
                break;
        }

        telemetry.addLine("Done with Jewel. Looking for Pictograph");
        telemetry.update();

        sleep(2000);

        cryptoboxKey = getPictographKey();

        turnRight_Encoder(25.0);
        sleep(1000);

        moveBackwardEncoder(2.0);

        switch (cryptoboxKey) {
            case LEFT:
                turnLeft_Encoder(160.0);
                break;
            case UNKNOWN:
            case CENTER:
                turnLeft_Encoder(150.0);
                break;
            case RIGHT:
                turnLeft_Encoder(135.0);
                break;
        }

        moveStraightTime(0.5, 1000);
        // Drops pre-loaded glyph into the cryptobox
        dropGlyph();
        moveBackwardEncoder(1.0, ENCODER_DRIVE_POWER);

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


    private void moveForwardEncoder(double distanceInFeet) {
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
        else
            moveForwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
    }

    private void moveBackwardEncoder(double distanceInFeet) {
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
        else
            moveBackwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
    }

    private void moveForwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveForwardRightEncoder(distanceInFeet, drivePower);
    }

    private void moveBackwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveBackwardRightEncoder(distanceInFeet, drivePower);
    }

    private void moveForwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() - (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backRightDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveForwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backLeftDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() - (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backLeftDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnRight_Encoder (double degreesOfTurn)
    {
        if (USE_LEFT_ENCODER)
            turnRight_LeftEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
        else
            turnRight_RightEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
    }

    private void turnLeft_Encoder (double degreesOfTurn)
    {
        if (USE_LEFT_ENCODER)
            turnLeft_LeftEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
        else
            turnLeft_RightEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
    }

    private void turnRight_Encoder (double degreesOfTurn, double ecPerDegree)
    {
        if (USE_LEFT_ENCODER)
            turnRight_LeftEncoder(degreesOfTurn, ecPerDegree);
        else
            turnRight_RightEncoder(degreesOfTurn, ecPerDegree);
    }

    private void turnLeft_Encoder (double degreesOfTurn, double ecPerDegree)
    {
        if (USE_LEFT_ENCODER)
            turnLeft_LeftEncoder(degreesOfTurn, ecPerDegree);
        else
            turnLeft_RightEncoder(degreesOfTurn, ecPerDegree);
    }

    private void turnRight_LeftEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backLeftDrive.getCurrentPosition();
        int targetPos = origPos + (int) (degreesOfTurn * ecPerDegree);
        setDrive(ENCODER_TURN_POWER, -ENCODER_TURN_POWER);
        while (backLeftDrive.getCurrentPosition() < targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnRight_RightEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backRightDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * ecPerDegree);
        setDrive(ENCODER_TURN_POWER, -ENCODER_TURN_POWER);
        while (backRightDrive.getCurrentPosition() > targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backRightDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backRightDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnLeft_LeftEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backLeftDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * ecPerDegree);
        setDrive(-ENCODER_TURN_POWER, ENCODER_TURN_POWER);
        while (backLeftDrive.getCurrentPosition() > targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnLeft_RightEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backRightDrive.getCurrentPosition();
        int targetPos = origPos + (int) (degreesOfTurn * ecPerDegree);
        setDrive(-ENCODER_TURN_POWER, ENCODER_TURN_POWER);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backRightDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backRightDrive.getCurrentPosition() - origPos, targetPos - origPos);
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
         else if(((double)colorSensor.red()) / ((double)colorSensor.blue()) > (1.0 + UNCERTAINTY)) {
            return ColorViewed.RED;

        } else if (((double)colorSensor.red()) / ((double)colorSensor.blue()) < (1.0 - UNCERTAINTY)) {
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

    private RelicRecoveryVuMark getPictographKey() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        return vuMark;
    }
    private void setupVuMarkData(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vParameters.vuforiaLicenseKey = "Ad/QI4f/////AAAAGTjpPxbdZUqSnVc3mldXKV0E3Ubo8UkPrp0l5P0ie1EXwbAiJNburExxvybAM/e5esxGLn3dl5zN73V9qcvBOROjy68/GQ8c0doo8ApEL127pzLSQEP6rZeq589EtDerLpgCqhsXSnU1hzLJ8S0UcgM9MeeUErzvfso6YAjLGZ9JXzLZHjXlX9lapHT64fBax9lZvMw5pmmZQE/j7oXqeamcdgnKyUn+wQN/3Gb+I2Ye7utY/LFJTiXFYesZsmE/eaq2mGnKVmqA4u6hvrSbEx/QudLqhl3nlXrAUPK+tx+5ersWNIB6OnNaRQApdYJb4mnO8qP8MgWhMIdT4Fuqd3WbitRP1NPUzqO+pJ63c36u";

        vParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vParameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }
}
