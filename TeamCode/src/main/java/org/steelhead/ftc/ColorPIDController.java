package org.steelhead.ftc;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Alec Matthews on 10/23/2016.
 * This class is a PID controller built for a color sensor.
 */

public class ColorPIDController {
    //initialize values
    private Thread pidThread;
    private int offsetValue;

    @Deprecated
    private boolean isOutputAvailable = false;

    private volatile double output = 0;
    private boolean isActive = true;
    private double kp;
    private double ki;
    private double kd;
    private double tolerance;

    private ElapsedTime logRate = new ElapsedTime();
    private String TAG;

    public ColorPIDController(final ColorSensor colorSensor, int thresholdLow, int thresholdHigh,
                              final String TAG) {
        this.TAG = TAG + ":color sensor";
        this.offsetValue = (thresholdLow + thresholdHigh)/2;

        //Setup the separate thread for calculating the values
        //This is in a separate thread so it doesn't slow down the main thread.
        pidThread = new Thread(new Runnable() {
            @Override
            public void run() {
                //Calculate the PID output
                double error;
                double average;
                double lastError = 0;
                double integral = 0;
                double derivative;
                while (isActive) {
                    average = (colorSensor.red() + colorSensor.green() + colorSensor.blue())/3;
                    error = average - offsetValue;
                    /*
                    Calculate the integral term. We are clamping it when the sign changes
                    when the error is 0 or when the error value is too big.
                    */
                    integral = integral + (error*0.017);
                    if (lastError > 0 && error < 0) {
                        integral = 0;
                    }else if (lastError < 0 && error > 0) {
                        integral = 0;
                    }else if (error == 0) {
                        integral = 0;
                    }
                    if (Math.abs(error) > 1) {
                        integral = 0;
                    }

                    derivative = ((error - lastError)/0.017);
                    lastError = error;

                    output = (kp * error) + (ki * integral) + (kd * derivative);

                    if (logRate.milliseconds() >= 500) {
                        logRate.reset();
                        Log.i(TAG, String.format("BRIGHTNESS: %f | OUTPUT: %f | OFFSET: %d",
                                average, output, offsetValue));
                    }
                    //Wait for the sensor to gather new values
                    //and slow down the loop so the integral term doesn't get too big too fast
                    try {
                        Thread.sleep(17);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
    }
    //This function is used to set the PID values. It must be run before the thread is started
    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    //Start the thread
    public void enable() {
        pidThread.start();
        logRate.reset();
    }

    //Stop the thread
    public void disable() {
        isActive = false;
    }
    @Deprecated
    public boolean isOutputAvailable() {
        return isOutputAvailable;
    }
    //Get the value of calculated by the PID controller
    public double getOutput() {
        if (Math.abs(output) < tolerance) {
            return 0;
        }
        return output;
    }
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
}
