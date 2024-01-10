package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.CenterStageVisualPipeline;

@TeleOp(name="Nothing But Can Break (Camera)")
public class cameraTestingOpmode extends LinearOpMode {
    private CenterStageVisualPipeline line = new CenterStageVisualPipeline(CenterStageVisualPipeline.PropColors.BLUE);
    @Override
    public void runOpMode(){
        centerStageRobot bot = new centerStageRobot(this);
        bot.getFrontCamera().setPipeline(line);
        sleep(3000);
        waitForStart();
        while(!isStopRequested()){
            bot.getMod().updateAprilTagData();
            if (bot.getMod().getAprilTagData() == null) continue;
            bot.writeToTelemetry("# of Detected Tags", bot.getMod().getAprilTagData().size());
            if(bot.getMod().getAprilTagData().size() > 0) {
                bot.writeToTelemetry("First Detected Tag:", bot.getMod().getAprilTagData().get(0).getId());
                bot.writeToTelemetry("First Detected Tag Distance:", bot.getMod().getAprilTagData().get(0).getDist());
            }
            bot.writeToTelemetry("Last Prop Location", line.getLastPropLocation());

            bot.updateTelemetry();
        }
    }
}
