package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;

@Autonomous(name="EncoderAutonomousBlueFar", group="CenterStage")
public class centerStageAutoBlueFar extends LinearOpMode {
    private CenterStageVisualPipeline line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.BLUE);
    private CenterStageVisualPipeline.PropLocation loc;
    private int iterations = 0;

    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        robot.getCamera().setPipeline(line);
        waitForStart();

        sleep(1000);
        while(line.getLastPropLocation() == CenterStageVisualPipeline.PropLocation.UNKNOWN && iterations < 500){ sleep(10); iterations++; }
        loc = line.getLastPropLocation();
        robot.driveInches(24, .5);
        // push thing in here
        if(loc == CenterStageVisualPipeline.PropLocation.LEFT){
            robot.turnDegrees(-90, .5);
            robot.driveInches(constants.SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.driveInches(-constants.SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.turnDegrees(90, .5);
        } else if(loc == CenterStageVisualPipeline.PropLocation.CENTER || loc == CenterStageVisualPipeline.PropLocation.UNKNOWN) {
            // center and fallback
            robot.driveInches(constants.CENTER_AUTO_PUSH_PIX_FORWARD_DIST_INCHES, .5);
        } else if(loc == CenterStageVisualPipeline.PropLocation.RIGHT) {
            robot.turnDegrees(90, .5);
            robot.driveInches(constants.SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.driveInches(-constants.SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.turnDegrees(-90, .5);
        }

        // sleep to avoid hitting our teammate
        sleep(constants.FAR_WAIT_TEAMMATE_MILLISECONDS);

        //go to back board
        robot.driveInches(-25, .5);
        robot.turnDegrees(-90, .3);
        robot.driveInches(85, .5);
    }
}
