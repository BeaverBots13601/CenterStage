package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;

import java.util.ArrayList;

@TeleOp(name="Camera Test")
public class cameraTestingOpmode extends LinearOpMode {
    private final CenterStageVisualPipeline line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.BLUE);
    @Override
    public void runOpMode(){
        centerStageRobot bot = new centerStageRobot(this);
        bot.getFrontCamera().setPipeline(line);
        sleep(3000);
        waitForStart();
        while(!isStopRequested()){
            ArrayList<AprilTagData> tags = bot.getMod().updateAprilTagData();
            if (tags == null) continue;
            bot.writeToTelemetry("# of Detected Tags", tags.size());
            if(tags.size() > 0) {
                bot.writeToTelemetry("First Detected Tag:", tags.get(0).getId());
                bot.writeToTelemetry("First Detected Tag Distance:", tags.get(0).getDist());
            }
            bot.writeToTelemetry("Last Prop Location", line.getLastPropLocation());

            bot.updateTelemetry();
        }
    }
}
