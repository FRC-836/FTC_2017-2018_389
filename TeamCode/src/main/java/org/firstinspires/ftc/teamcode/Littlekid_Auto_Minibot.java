package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.DalvCode;

@Autonomous(name = "Lil Timmy",group = "")
public class Littlekid_Auto_Minibot extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotor.class,"left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);

        wait(1000);

        leftDrive.setPower(1.0);
        rightDrive.setPower(-1.0);

        wait(1000);

        leftDrive.setPower(-1.0);

        wait(800);

        rightDrive.setPower(1);

        wait(400);

        leftDrive.setPower(1);

        wait(200);

        rightDrive.setPower(-0.5);

        wait(1000);
    }


}

