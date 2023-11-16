package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;

@Autonomous(name="Nothing But Can Break")
public class cameraTestingOpmode extends LinearOpMode {
    private CenterStageVisualPipeline line = new CenterStageVisualPipeline();
    @Override
    public void runOpMode(){
        waitForStart();
        centerStageRobot bot = new centerStageRobot(this);
        bot.getCamera().setPipeline(line);
        while(!isStopRequested()){
            bot.writeToTelemetry("Last Result", line.getLastResult());
            bot.updateTelemetry();
        }
    }
}
