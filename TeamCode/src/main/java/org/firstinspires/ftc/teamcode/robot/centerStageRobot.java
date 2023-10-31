package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Arrays;

@Disabled
public abstract class centerStageRobot extends baseRobot {
    public centerStageRobot() {
        super(3, 15);
    }


    public void driveStrafe(double inches, double power) {
        int[] target = new int[this.driveMotors.length];
        //TODO: test this and make sure that these are the right changes
        double[] powers = new double[] {power, -power, -power, power};
        Arrays.fill(target, (int) this.inchesToEncoder(inches));

        driveEncoded(target, powers);
    }
}
