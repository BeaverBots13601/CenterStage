package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
@Autonomous(name="EncoderAutonomous", group="CenterStage")
public class centerStageAuto extends centerStageRobot {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (opModeIsActive()) {
            this.driveInches(10, .5);
            this.turnDegrees(90, .3);
            this.driveInches(10, .5);
            this.turnDegrees(90, .3);
            this.driveInches(10, .5);
            this.turnDegrees(90, .3);
            this.driveInches(10, .5);
            this.turnDegrees(90, .3);

            this.driveStrafe(10, .5);
        }
    }
}
