package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.structures.AprilTagData;

import java.util.Arrays;

public class centerStageRobot extends baseRobot {
    public centerStageRobot(LinearOpMode opmode) {
        super(opmode, 3.5, 13.75);
    }

    public void driveStrafe(double inches, double power) {
        int ticks = (int) this.inchesToEncoder(inches);
        int[] target = new int[] {ticks, -ticks, -ticks, ticks};
        //TODO: test this and make sure that these are the right changes
        double[] powers = new double[] {power, -power, -power, power};

        driveEncoded(target, powers);
    }

    public AprilTagData[] identifyTags() {
        return new AprilTagData[] {};
    }
    // alaister do packing list & physical field mode switch
}
