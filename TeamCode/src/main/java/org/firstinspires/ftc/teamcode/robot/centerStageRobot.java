package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.structures.AprilTagData;

import java.util.Arrays;

public class centerStageRobot extends baseRobot {
    private DcMotorEx armMotor;
    private double holdMotorPower = 0.2;

    public centerStageRobot(LinearOpMode opmode) {
        super(opmode, 3.5, 13.75);

        //this.armMotor = createDefaultMotor("armMotor");
    }

    public void driveStrafe(double inches, double power) {
        int ticks = (int) this.inchesToEncoder(inches);
        int[] target = new int[] {ticks, -ticks, -ticks, ticks};
        double[] powers = new double[] {power, -power, -power, power};

        driveEncoded(target, powers);
    }

    public AprilTagData[] identifyTags() {
        return new AprilTagData[] {};
    }

    // servo specs: GoBilda 2000 Series Dual Mode Servo (25-3, Speed)
    // SKU: 2000-0025-0003


}
