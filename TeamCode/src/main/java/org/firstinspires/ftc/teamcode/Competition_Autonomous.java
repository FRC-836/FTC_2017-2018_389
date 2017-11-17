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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
public class Competition_Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;
    private DigitalChannel glyphHolder = null;

    private final double BEEP_EC_PER_FEET = 1950.0;
    private final double BEEP_EC_PER_DEGREES = 92.2 / 4.0;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        intakeRight = hardwareMap.get(DcMotor.class, "intake_right");
        intakeLeft = hardwareMap.get(DcMotor.class, "intake_left");
        glyphHolder = hardwareMap.get(DigitalChannel.class, "glyph_holder");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        // These two steps move the robot from the red platform to the red goal.

        moveStraightRightEncoder(2.9);
        turnRightEncoder(90);
        dropGlyphIntoCryptobox();
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

    private void setIntake(double intakePower) {
        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);
        intakeRight.setPower(intakePower);
        intakeLeft.setPower(intakePower);
    }

    private void moveStraightLeftEncoder(double distanceInFeet) {
        int targetPos = backLeftDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(0.5, 0.5);
        while (backLeftDrive.getCurrentPosition() < targetPos) ;
        setDrive(0.0, 0.0);
    }
    private void moveStraightRightEncoder(double distanceInFeet) {
        int targetPos = backRightDrive.getCurrentPosition() + (int)(distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(0.5, 0.5);
        while(backRightDrive.getCurrentPosition()<targetPos);
        setDrive(0.0, 0.0);
    }

    private void turnRightEncoder(double degreesOfTurn) {
        int origPos = frontLeftDrive.getCurrentPosition();
        int targetPos = origPos - (int)(degreesOfTurn * BEEP_EC_PER_DEGREES);
        setDrive(1.0, -1.0);
        while(frontLeftDrive.getCurrentPosition()<targetPos)
        {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, frontLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, frontLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }
    private void dropGlyphIntoCryptobox() {

    }
}
