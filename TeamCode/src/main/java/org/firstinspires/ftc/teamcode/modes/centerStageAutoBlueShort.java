package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;

@Autonomous(name="EncoderAutonomousBlueClose", group="CenterStage")
public class centerStageAutoBlueShort extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        waitForStart();

        if (opModeIsActive()) {
            robot.driveInches(24, .5);
            // push thing in here
            robot.driveInches(constants.AUTO_PUSH_PIX_FORWARD_DIST_INCHES, .5);

            robot.driveInches(-21, .5);
            robot.turnDegrees(-90, .5);
            robot.driveInches(24, .5);
            robot.turnDegrees(90, .5);
            robot.driveInches(41, .5);
            robot.turnDegrees(-90, .5);
            robot.driveInches(22, .5);
        }
    }
}
