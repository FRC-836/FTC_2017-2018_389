package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by user on 7/10/2018.
 */

public class PI_Controller {
    ElapsedTime runtime;

    private int setpoint = 0;
    private int error = 0;
    private int lastError = 0;
    private double time = 0;
    private double lastTime = 0;
    private double pValue = 0;
    private double PGAIN;
    private double iValue = 0;
    private double IGAIN;
    private boolean isFirstTime = true;
    //private double CONVERSION_RATE = conversionRate;


}