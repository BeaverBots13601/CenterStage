package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.RobotClasses.BasicRobot;

@Autonomous(name="Motor Encoder Test")
public class BasicLauncherOpmode extends LinearOpMode {
    private BasicRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BasicRobot(hardwareMap, telemetry, 3);
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Started");
        telemetry.update();
        robot.runOpMode();

        while(opModeIsActive()){

        }

    }
}
