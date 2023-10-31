package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Disabled
public class centerStageRobot extends baseRobot {
    public centerStageRobot(LinearOpMode opmode) {
        super(opmode, 3.5, 13.75);
    }


    public void driveStrafe(double inches, double power) {
        int[] target = new int[this.driveMotors.length];
        //TODO: test this and make sure that these are the right changes
        double[] powers = new double[] {power, -power, -power, power};
        Arrays.fill(target, (int) this.inchesToEncoder(inches));

        driveEncoded(target, powers);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
