package org.firstinspires.ftc.teamcode.LegacyClasses;

public class EncoderPositions {
    private final double leftFront;
    private final double leftBack;
    private final double rightFront;
    private final double rightBack;
    public EncoderPositions(double leftFront, double leftBack, double rightFront, double rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }
    public double getLeftFront() {
        return leftFront;
    }
    public double getLeftBack() {
        return leftBack;
    }
    public double getRightFront() {
        return rightFront;
    }
    public double getRightBack() {
        return rightBack;
    }
}
