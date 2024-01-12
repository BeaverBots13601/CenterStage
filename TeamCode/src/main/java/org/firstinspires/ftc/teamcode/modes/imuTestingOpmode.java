package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;

@Autonomous(name="IMU Turning Test")
@Disabled
public class imuTestingOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        waitForStart();
        robot.turnImuDegrees(90, .4);
        sleep(5000);
        robot.turnImuDegrees(-180, .4);
        sleep(5000);
        robot.turnImuDegrees(90, .4);
    }
}

