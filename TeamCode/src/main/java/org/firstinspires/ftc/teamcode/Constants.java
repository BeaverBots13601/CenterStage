package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final String[] DRIVE_MOTOR_NAMES = {"leftFront", "leftBack", "rightFront", "rightBack"};
    public static final String[] LAUNCHER_MOTOR_NAMES = {"leftBigWheel", "rightBigWheel", "gearMotor"};

    //Robot stuff
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    // todo Ben math. Understand later
    public static final double MOTOR_TICKS_PER_REV = 537.70;
    public static final double WHEEL_DIAMETER = 3.78;
    public static final double TICKS_PER_INCH = MOTOR_TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
    public static final double PID_INCHES_TOLERANCE = 1;
    public static final double PID_ANGLE_TOLERANCE = Math.toRadians(1);
    public static final double AUTONOMOUS_DRIVE_SPEED = .65;
    public static final double AUTO_POWER_STEP = .05;
// TODO: make sure this is still accurate
    public static final double ROBOT_CIRCUMFERENCE = 98.19;


    // todo last years pids, recalibrate and update
    public static double KP = 0.225;
    public static double KI = 0;
    public static double KD = 0.0325;
    public static double X_KP = 0.225;
    public static double X_KI = 0;
    public static double X_KD = 0.03;
    public static double ANGLE_KP = 0.250;
    public static double ANGLE_KI = 0;
    public static double ANGLE_KD = 0.046;
}
