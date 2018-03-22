package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Lift PID", group="Main")
public class Lift_PID extends Autonomous_Parent {

    private final double EC_PER_DEGREE_LIFT = 8.475;

    @Override
    public void runAutonomous() {
        waitForStart();
        PID_Loop liftLoop = new PID_Loop(0.001, 0.0003, 0.0, EC_PER_DEGREE_LIFT);
        liftLoop.setGoal(0.0);
        resetArmEncoder();

        while (opModeIsActive())
        {
            double liftPower = liftLoop.update(liftMotor.getCurrentPosition());
            liftMotor.setPower(liftPower);
            if (gamepad1.a)
                liftLoop.setGoal(0.0);
            if (gamepad1.b)
                liftLoop.setGoal(45.0);
            if (gamepad1.x)
                liftLoop.setGoal(90.0);
            if (gamepad1.y)
                liftLoop.setGoal(135.0);
        }
    }
}