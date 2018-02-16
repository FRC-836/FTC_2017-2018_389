package org.firstinspires.ftc.teamcode;

public class Teleop_Parent extends Robot_Parent
{
    protected boolean isModeFast = true;

    private final double JOYSTICK_THRESHOLD = 0.1;
    protected final double SLOW_DRIVE_SCALE_FACTOR = 0.5;
    protected final double LIFT_POWER_DOWN = -0.10;
    protected final double LIFT_POWER_IDLE = 0.16;//Original arm: 0.09, double arm originally 0.18-too high
    protected final double LIFT_POWER_UP = 0.50;
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
        setJewelArm(JEWEL_ARM_FULLY_UP);
        sleep(250);
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