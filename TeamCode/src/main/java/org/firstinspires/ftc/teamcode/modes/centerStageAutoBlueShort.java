package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline.PropLocation;

@Autonomous(name="EncoderAutonomousBlueClose", group="CenterStage")
public class centerStageAutoBlueShort extends LinearOpMode {
    private CenterStageVisualPipeline line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.BLUE);
    private PropLocation loc;
    private int iterations = 0;

    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        robot.getFrontCamera().setPipeline(line);
        waitForStart();
        sleep(1000);

        while(line.getLastPropLocation() == PropLocation.UNKNOWN && iterations < 500){ sleep(10); iterations++; }
        loc = line.getLastPropLocation();
        robot.driveInches(26, .25);
        // push thing in here
        if(loc == PropLocation.LEFT){
            robot.turnDegrees(-89, .25);
            robot.driveInches(constants.LEFT_SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.driveInches(-constants.LEFT_SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.turnDegrees(89, .25);
        } else if(loc == PropLocation.CENTER || loc == PropLocation.UNKNOWN) {
            // center and fallback
            robot.driveInches(constants.CENTER_AUTO_PUSH_PIX_FORWARD_DIST_INCHES, .5);
        } else if(loc == PropLocation.RIGHT) {
            robot.turnDegrees(89, .25);
            robot.driveInches(2.5, .5);
            robot.driveInches(-4.5, .5);
            robot.turnDegrees(-89, .25);
        }

        robot.driveInches(-22, .5);
        robot.turnDegrees(-90, .5);
        robot.driveInches(24, .5);
        robot.turnDegrees(90, .5);
        robot.driveInches(41, .5);
        robot.turnDegrees(-90, .5);
        robot.driveInches(18, .5);
    }
}
