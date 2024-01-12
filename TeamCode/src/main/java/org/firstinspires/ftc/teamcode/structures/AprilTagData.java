package org.firstinspires.ftc.teamcode.structures;

public class AprilTagData {
    private int id;
    private final double dist;
    private int correctedBits;
    public AprilTagData(int id, double dist, int correctedBits){
        this.id = id;
        this.dist = dist;
        this.correctedBits = correctedBits;
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
}
