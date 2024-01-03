package org.firstinspires.ftc.teamcode.structures;

import org.opencv.core.Point;

public class AprilTagData {
    private int id;
    private double dist;
    private int correctedBits;
    private Point[] corners;
    private Point center;
    public AprilTagData(int id, double dist, int correctedBits, Point[] corners, Point center){
        this.id = id;
        this.dist = dist;
        this.correctedBits = correctedBits;
        this.corners = corners;
        this.center = center;
    }

    public AprilTagData(){
        this.dist = -1;
    }

    public int getId() {
        return id;
    }
    public double getDist() {
        return dist;
    }
    public int getCorrectedBits() {
        return correctedBits;
    }
    public Point[] getCorners() {
        return corners;
    }
    public Point getCenter() {
        return center;
    }
}
