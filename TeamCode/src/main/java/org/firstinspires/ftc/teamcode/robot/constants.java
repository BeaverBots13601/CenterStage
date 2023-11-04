package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {
    public static int FAR_WAIT_TEAMMATE_MILLISECONDS = 7500;

    public enum driveMotorName {
        leftFront, leftBack, rightFront, rightBack;

    }
    public enum OrientationMode {
        FIELD,
        ROBOT
    }
    public static final double ENCODER_TICKS = 537.70;
    public static final int TELEMETRY_MS_TRANSMISSION_INTERVAL = 25;

    public static double AUTO_PUSH_PIX_FORWARD_DIST_INCHES = 4.5;

    public enum SPEEDS {
        NORMAL,
        FAST,
        CUSTOM_FTC_DASHBOARD
    }

    public static final double NORMAL_SPEED = 0.65;
    public static final double FAST_SPEED = 0.85;
    public static double CUSTOM_FTC_DASHBOARD_SPEED = 0.65;
}
