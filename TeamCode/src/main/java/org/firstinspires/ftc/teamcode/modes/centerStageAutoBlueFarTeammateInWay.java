package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;

@Disabled
@Autonomous(name="AutonomousBlueFarTeammateInCorner", group="CenterStage")
public class centerStageAutoBlueFarTeammateInWay extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        waitForStart();

        if (opModeIsActive()) {
            //gets to team prop location area
            robot.driveInches(24, .5);
            sleep(1000);
            //fill in area with code for detecting prop and putting pixel
            robot.driveInches(constants.AUTO_PUSH_PIX_FORWARD_DIST_INCHES, .3);

            // sleep to avoid hitting our teammate
            sleep(constants.FAR_WAIT_TEAMMATE_MILLISECONDS);

            //go to back board
            robot.driveInches(-25, .5);
            robot.turnDegrees(-90, .3);
            robot.driveInches(61, .5);

            // move to not corner
            robot.turnDegrees(90, .5);
            robot.driveInches(41, .5);
            robot.turnDegrees(-90, .5);
            robot.driveInches(22, .5);
        }
    }
}
