package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 4/21/2018.
 */

@Autonomous(name="Encoder_Tester",group="Main")
public class Encoder_Tester extends Autonomous_Parent{
    @Override
    public void runAutonomous() {
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && !gamepad1.a){
            jewelArm.setPosition(gamepad1.right_stick_x/2.0 + 0.5);
            telemetry.addData("jewel arm", jewelArm.getPosition());
            telemetry.update();
        }
        moveStraightTime(1.0, 3000);
        while(opModeIsActive()) {
            telemetry.addData("Encoder counts left", backLeftDrive.getCurrentPosition());
            telemetry.addData("Encoder counts right", backRightDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
