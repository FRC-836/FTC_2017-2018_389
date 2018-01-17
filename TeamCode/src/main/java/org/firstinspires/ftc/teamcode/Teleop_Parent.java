package org.firstinspires.ftc.teamcode;

public class Teleop_Parent extends Robot_Parent
{
    protected boolean isModeFast = true;

    private final double JOYSTICK_THRESHOLD = 0.1;
    protected final double SLOW_DRIVE_SCALE_FACTOR = 0.5;

    @Override
    public void initializeRobot() {

    }

    @Override
    public void startRobot() {
        raiseJewelArm();
        while (opModeIsActive())
        {
            cycle();
        }
    }

    public void cycle() {

    }

    protected double controllerThreshold(double number){
        if (Math.abs(number) <= JOYSTICK_THRESHOLD) {
            return 0.0;
        }
        else {
            return number;
        }
    }
}