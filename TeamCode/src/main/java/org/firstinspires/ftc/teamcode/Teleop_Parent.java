package org.firstinspires.ftc.teamcode;

public class Teleop_Parent extends Robot_Parent
{
    protected boolean isModeFast = true;

    private final double JOYSTICK_THRESHOLD = 0.1;
    protected final double SLOW_DRIVE_SCALE_FACTOR = 0.5;
    protected final double LIFT_POWER_UP = 1.0;
    protected final double LIFT_POWER_DOWN = -0.3;
    protected final double LIFT_POWER_HOLD_GUESS = 0.2;
    protected final double PICK_UP_GLYPH_POWER = 1.0;
    protected final double DROP_GLYPH_POWER = -1.0;

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