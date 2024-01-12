package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CenterStageAprilTagModule;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline.PropLocation;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;

@Autonomous(name="EncoderAutonomousUnified")
public class centerStageAutoUnified extends LinearOpMode {
    private enum Locations {
        BlueClose,
        BlueFar,
        RedClose,
        RedFar
    }
    private CenterStageVisualPipeline line;
    private PropLocation propLocation;
    private int iterations = 0;
    private Locations currentLocation;
    public void runOpMode(){
        centerStageRobot robot = new centerStageRobot(this);
        robot.getFrontCamera().setPipeline(line);

        CenterStageAprilTagModule tags = robot.getMod();
        tags.updateAprilTagData();
        AprilTagData max = new AprilTagData(); // default
        for(AprilTagData tag : tags.getAprilTagData()){
            if(Math.max(tag.getDist(), max.getDist()) == max.getDist()) max = tag;
        }

        sleep(1000);

        // assumes camera is mounted on left side. Sorry it's kinda confusing, using a map helps to understand
        if (max.getId() == 7 || max.getId() == 10) { // sees red wall tag
            if(max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES){
                currentLocation = Locations.RedClose; // tag far away, we are close to bb
            } else {
                currentLocation = Locations.RedFar; // tag nearby
            }
        } else {
            if (max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                currentLocation = Locations.BlueFar; // inverse of blue because the camera is pointing at & reading bb now
            } else {
                currentLocation = Locations.BlueClose; // tag nearby
            }
        }

        if(currentLocation == Locations.BlueClose || currentLocation == Locations.BlueFar) {
            line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.BLUE);
        } else {
            line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.RED);
        }

        waitForStart();
        sleep(1000);

        while(line.getLastPropLocation() == PropLocation.UNKNOWN && iterations < 500){ sleep(10); iterations++; }
        propLocation = line.getLastPropLocation();
        robot.driveInches(26, .25);
        // push thing in here
        if(propLocation == PropLocation.LEFT){
            robot.turnDegrees(-89, .25);
            robot.driveInches(constants.LEFT_SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.driveInches(-constants.LEFT_SIDE_AUTO_PUSH_PIX_INTO_POS_DIST_INCHES, .5);
            robot.turnDegrees(89, .25);
        } else if(propLocation == PropLocation.CENTER || propLocation == PropLocation.UNKNOWN) {
            // center and fallback
            robot.driveInches(constants.CENTER_AUTO_PUSH_PIX_FORWARD_DIST_INCHES, .5);
        } else if(propLocation == PropLocation.RIGHT) {
            robot.turnDegrees(89, .25);
            robot.driveInches(2.5, .5);
            robot.driveInches(-4.5, .5);
            robot.turnDegrees(-89, .25);
        }

        switch(currentLocation){
            case BlueClose: {
                robot.driveInches(-22, .5);
                robot.turnDegrees(-90, .5);
                robot.driveInches(24, .5);
                robot.turnDegrees(90, .5);
                robot.driveInches(41, .5);
                robot.turnDegrees(-90, .5);
                robot.driveInches(18, .5);
            }
            case BlueFar: {
                // sleep to avoid hitting our teammate
                sleep(constants.FAR_WAIT_TEAMMATE_MILLISECONDS);

                //go to back board
                robot.driveInches(-25, .5);
                robot.turnDegrees(90, .3);
                robot.driveInches(85, .5);
            }
            case RedClose: {
                robot.driveInches(-22, .5);
                robot.turnDegrees(90, .5);
                robot.driveInches(24, .5);
                robot.turnDegrees(-90, .5);
                robot.driveInches(41, .5);
                robot.turnDegrees(90, .5);
                robot.driveInches(18, .5);
            }
            case RedFar: {
                // sleep to avoid hitting our teammate
                sleep(constants.FAR_WAIT_TEAMMATE_MILLISECONDS);

                //go to back board
                robot.driveInches(-25, .5);
                robot.turnDegrees(-90, .3);
                robot.driveInches(85, .5);
            }
            default: {
                return;
            }
        }
    }
}
