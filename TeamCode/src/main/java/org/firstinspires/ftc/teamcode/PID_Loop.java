package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Loop {

    ElapsedTime runtime;

    private int setpoint = 0;
    private int error = 0;
    private double time = 0.0;
    private int lastError = 0;
    private double lastTime = 0.0;
    private double pValue = 0.0;
    private double iValue = 0.0;
    private double dValue = 0.0;
    private double PGAIN;
    private double IGAIN;
    private double DGAIN;
    private boolean isFirstTime = true;

    private double CONVERSION_FACTOR;

    public PID_Loop(double pGain, double iGain, double dGain, double conversionFactor) {
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.PGAIN = pGain;
        this.IGAIN = iGain;
        this.DGAIN = dGain;
        this.CONVERSION_FACTOR = conversionFactor;
    }

    public void setGoal(double goal) {
        this.setpoint = (int) Math.round(goal * CONVERSION_FACTOR);
    }

    public double update(int input)
    {
        lastError = error;
        error = setpoint - input;
        lastTime = time;
        time = runtime.seconds();

        // pValue
        pValue = PGAIN * (double) error;

        //iValue
        iValue += IGAIN * (double) (lastError + error) * (0.5) * (time - lastTime);

        //dValue
        dValue = DGAIN * (double) (error - lastError) / (time - lastTime);

        if (isFirstTime)
        {
            iValue = 0.0;
            dValue = 0.0;
            isFirstTime = false;
        }
        return pValue + iValue + dValue;
    }

    public void resetPID() {
        resetPID(0.0);
    }

    public void resetPID(double startingIValue) {
        runtime.reset();
        isFirstTime = true;
        iValue = startingIValue;
    }
}