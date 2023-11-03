package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Nothing")
public class doNothingAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        sleep(25000);
    }
}
