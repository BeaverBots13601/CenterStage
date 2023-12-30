package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {
    public static int FAR_WAIT_TEAMMATE_MILLISECONDS = 7500;

    public enum driveMotorName {
        leftFront, leftBack, rightFront, rightBack

    }
    public enum OrientationMode {
        FIELD,
        ROBOT
    }
    public static final double ENCODER_TICKS = 537.70;
    public static final int TELEMETRY_MS_TRANSMISSION_INTERVAL = 25;

    // Autonomous constants
    public static double CENTER_AUTO_PUSH_PIX_FORWARD_DIST_INCHES = 4;
    public static double LEFT_SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES = 4;
    //public static double AUTO_BACKUP_EXTRA_DIST = 4;

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
    // Default speed mode is set here; we can adjust it from FTCDashboard this way
    public static SPEEDS currentSpeedMode = SPEEDS.NORMAL;

    public static final String KNOCKER_SERVO_NAME = "knockerServo";
    public static final String PAL_SERVO_NAME = "PALServo";
    public static final String GRAPPLE_SERVO_NAME = "grappleServo";

    /**
     * The time we wait and maintain motor power after the robot has successfully hung, in MS
     */
    // 5 min currently
    public static int GRAPPLE_WAIT_TIME_MS = 300000;

    // Camera data
    public static final String FRONT_CAMERA_NAME = "camera";
    public static final int FRONT_CAMERA_WIDTH = 1280;
    public static final int FRONT_CAMERA_HEIGHT = 720;

    public static final String SIDE_CAMERA_NAME = "sideCamera";
    public static final int SIDE_CAMERA_WIDTH = 1280;
    public static final int SIDE_CAMERA_HEIGHT = 720;

    /**
     * The percentage, from [0, 1.0], at which values equal to or below will not be used to determine the location of the team prop.
     * Ex. If the highest percentage of color is the center area at 8%, and the value is at 0.1, it will be declared 'unknown' instead of 'center'.
     */
    public static double COLOR_UNKNOWN_THRESHOLD_PERCENT = 0.1;
    public static double DETECTION_BOX_OFFSET_SIDES_PX = 128;

    public static final double ROBOT_CIRCUMFERENCE = 98.19;

    // found this might help? https://pidexplained.com/how-to-tune-a-pid-controller/
    public static double ANGLE_KP = 0.250;

    public static double ANGLE_KD = 0.046;

    // temporary tuning values
    public static double a = .78;
    public static double b = .3;
}
