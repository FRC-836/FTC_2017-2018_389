package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Loop {

    ElapsedTime runtime;

    private double setpoint = 0;
    private double error = 0;
    private double time = 0.0;
    private double lastError = 0;
    private double lastTime = 0.0;
    private double pValue = 0.0;
    private double iValue = 0.0;
    private double dValue = 0.0;
    private double PGAIN;
    private double IGAIN;
    private double DGAIN;
    private boolean isFirstTime = true;

    public PID_Loop(double pGain, double iGain, double dGain) {
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.PGAIN = pGain;
        this.IGAIN = iGain;
        this.DGAIN = dGain;
    }

    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
    }

    public double update(double input)
    {
        lastError = error;
        error = setpoint - input;
        lastTime = time;
        time = runtime.seconds();

        // pValue
        pValue = PGAIN * error;

        //iValue
        iValue += IGAIN * (lastError + error) * (0.5) * (time - lastTime);

        //dValue
        dValue = DGAIN * (error - lastError) / (time - lastTime);

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