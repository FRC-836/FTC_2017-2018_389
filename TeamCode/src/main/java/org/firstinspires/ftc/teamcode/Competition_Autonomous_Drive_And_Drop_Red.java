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

@Autonomous(name="LAST RESORT Red Left Turn", group="Backup Drive")
public class Competition_Autonomous_Drive_And_Drop_Red extends Autonomous_Parent {

    @Override
    public void startRobot() {
        moveStraightTime(0.7, 1300);
        sleep(1000);
        setDrive(0.5, -0.5);
        sleep(1000);
        setDrive(0.0, 0.0);
        sleep(1000);
        moveStraightTime(0.7, 2000);
        dropGlyph();
        moveStraightTime(-0.35, 500);
    }
}