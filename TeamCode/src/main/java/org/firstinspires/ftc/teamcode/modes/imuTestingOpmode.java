package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;

@Autonomous
public class imuTestingOpmode extends LinearOpMode {
    private centerStageRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new centerStageRobot(this);
        waitForStart();
        robot.turnImuDegrees(90, .5, false);
        sleep(5000);
        robot.turnImuDegrees(-180, .5, false);
        sleep(5000);
        robot.turnImuDegrees(90, .5, false);
    }
}

