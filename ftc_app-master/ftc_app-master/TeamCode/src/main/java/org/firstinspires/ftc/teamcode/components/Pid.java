package org.firstinspires.ftc.teamcode.components;

/**
 * Created by 21maffetone on 2/22/18.
 */

/**
 * PID Controller: kp * (e + (integral(e) / ti) + (td * derivative(e))).
 * https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
 */
public class Pid {
    /**
     * Creates a PID Controller.
     * @param kp Proportional factor to scale error to output.
     * @param ti The number of seconds to eliminate all past errors.
     * @param td The number of seconds to predict the error in the future.
     * @param integralMin The min of the running integral.
     * @param integralMax The max of the running integral.
     * @param outputMin The min of the PID output.
     * @param outputMax The max of the PID output.
     */
    public Pid(double kp, double ti, double td, double integralMin,
               double integralMax, double outputMin, double outputMax) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
        this.outputMin = outputMin;
        this.outputMax = outputMax;

        this.previousError = 0;
        this.runningIntegral = 0;
    }

    /**
     * Performs a PID update and returns the output control.
     * @param desiredValue The desired state value (e.g. speed).
     * @param actualValue The actual state value (e.g. speed).
     * @param dt The amount of time (sec) elapsed since last update.
     * @return The output which impacts state value (e.g. motor throttle).
     */
    public double update(double desiredValue, double actualValue, double dt) {
        double e = desiredValue - actualValue;
        runningIntegral = clampValue(runningIntegral + e * dt,
                integralMin, integralMax);
        double d = (e - previousError) / dt;
        double output = clampValue(kp * (e + (runningIntegral / ti) + (td * d)),
                outputMin, outputMax);

        previousError = e;
        return output;
    }

    /**
     * Clamps a value to a given range.
     * @param value The value to clamp.
     * @param min The min clamp.
     * @param max The max clamp.
     * @return The clamped value.
     */
    public static double clampValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    // Proportional factor to scale error to output.
    private double kp;
    // The number of seconds to eliminate all past errors.
    private double ti;
    // The number of seconds to predict the error in the future.
    private double td;
    // The min of the running integral.
    private double integralMin;
    // The max of the running integral.
    private double integralMax;
    // The min allowed PID output.
    private double outputMin;
    // The max allowed PID output.
    private double outputMax;

    // The last error value.
    private double previousError;
    // The discrete running integral (bounded by integralMax).
    private double runningIntegral;
}
