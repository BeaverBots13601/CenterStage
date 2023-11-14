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

    // todo find way to condense these into single enum? or otherwise make not bad
    public enum SPEEDS {
        NORMAL,
        FAST,
        SLOW,
        CUSTOM_FTC_DASHBOARD
    }
    public static final double SLOW_SPEED = 0.4;
    public static final double NORMAL_SPEED = 0.65;
    public static final double FAST_SPEED = 0.80;
    // This speed is designed to be set within FTC Dashboard.
    public static double CUSTOM_FTC_DASHBOARD_SPEED = 0.65;
    public static SPEEDS currentSpeedMode = SPEEDS.NORMAL;
}
