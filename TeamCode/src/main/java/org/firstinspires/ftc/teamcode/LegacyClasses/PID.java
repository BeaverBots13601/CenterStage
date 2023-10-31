package org.firstinspires.ftc.teamcode.LegacyClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Proportional Integral Derivative Controller
 * - Allows for smooth transitions into a desired state.
 * - Used extensively in autonomous movement.
 */
public class PID {
    protected double Kp;
    protected double Ki;
    protected double Kd;

    protected double derivative;
    protected double integralSum;
    protected double lastError;
    protected ElapsedTime timer;

    public PID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.timer = new ElapsedTime();
        reset();
    }

    public double calculate(double current, double target) {
        double error = target - current;
        derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double result = Kp * error + Ki * integralSum + Kd * derivative;

        lastError = error;
        timer.reset();
        return result;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public double getLastError(){
        return lastError;
    }
    public double getDerivative() {
        return derivative;
    }
    public double getIntegralSum() {
        return integralSum;
    }
}
