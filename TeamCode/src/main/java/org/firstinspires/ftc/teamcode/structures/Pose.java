package org.firstinspires.ftc.teamcode.structures;

/**
 * Object to conveniently manage position data.
 */
public class Pose {
    private final double x;
    private final double y;
    private final double angle;

    public Pose() {
        this.x = 0;
        this.y = 0;
        this.angle = 0;
    }

    public Pose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngle() {
        return angle;
    }
}
