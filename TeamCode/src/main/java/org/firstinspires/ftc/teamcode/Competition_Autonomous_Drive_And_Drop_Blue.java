package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Backup Drive - Blue Turn", group="Backup Drive")
public class Competition_Autonomous_Drive_And_Drop_Blue extends Autonomous_Parent {

    @Override
    public void startRobot() {
        moveStraightTime(0.7, 1300);
        sleep(1000);
        setDrive(-0.5, 0.5);
        sleep(1000);
        setDrive(0.0, 0.0);
        sleep(1000);
        moveStraightTime(0.7, 2000);
        dropGlyph();
        moveStraightTime(-0.35, 500);
    }
}