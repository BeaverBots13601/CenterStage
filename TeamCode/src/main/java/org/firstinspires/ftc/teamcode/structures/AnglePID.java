package org.firstinspires.ftc.teamcode.structures;

import org.firstinspires.ftc.teamcode.robot.constants;
import org.firstinspires.ftc.teamcode.structures.PID;

/**
 * PID Controller which can handle angles.
 */
public class AnglePID extends PID {
    public AnglePID(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    @Override
    public double calculate(double current, double target) {
        double angleError = target - current;
//        Normalizes the error to account for angle wrap
        while (angleError > Math.PI) {
            angleError -= 2 * Math.PI;
        }
        while (angleError < -Math.PI) {
            angleError += 2 * Math.PI;
        }
        double error = angleError / (2 * Math.PI) * constants.ROBOT_CIRCUMFERENCE;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double result = Kp * error + Ki * integralSum + Kd * derivative;

        lastError = error;
        timer.reset();
        return result;
    }
}
