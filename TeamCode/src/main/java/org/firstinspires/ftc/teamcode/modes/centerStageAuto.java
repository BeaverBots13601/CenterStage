package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
@Autonomous(name="EncoderAutonomous", group="CenterStage")
public class centerStageAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        waitForStart();

        if (opModeIsActive()) {
            robot.setDriveMotors(new double[] {0, .5, 0, 0}, DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.update();

            int distance = 520;
            robot.driveEncoded(new int[] {distance, distance, distance, distance}, new double[] {.5,.5,.5,.5});
            robot.driveInches(10, .5);
            robot.turnDegrees(90, .5);
            robot.driveInches(10, .5);
            robot.turnDegrees(90, .3);
            robot.driveInches(10, .5);
            robot.turnDegrees(90, .3);
            robot.driveInches(10, .5);
            robot.turnDegrees(90, .3);

//            robot.driveStrafe(10, .5);
            sleep(5000);
        }
    }
}
