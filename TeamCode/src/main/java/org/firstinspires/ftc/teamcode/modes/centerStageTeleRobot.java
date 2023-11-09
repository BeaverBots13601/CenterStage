package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;

@TeleOp(name="TeleOp Controls (Robot)", group ="CenterStage")
public class centerStageTeleRobot extends LinearOpMode {
    private centerStageRobot robot;
    private Gamepad previousGamepad;
    private constants.SPEEDS currentSpeed = constants.SPEEDS.NORMAL;

    public void runOpMode() {
        robot = new centerStageRobot(this);
        int tmp_deadzoneadjust = 2;

        waitForStart();


        while(opModeIsActive()){
            Gamepad currentGamepad = gamepad1;

            if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
                // shift up if not already at highest speed. If in custom FTC dashboard, go to normal
                switch(currentSpeed){
                    case FAST:
                    case NORMAL:
                        currentSpeed = constants.SPEEDS.FAST;
                        break;
                    case CUSTOM_FTC_DASHBOARD:
                    case SLOW:
                        currentSpeed = constants.SPEEDS.NORMAL;
                        break;
                }
            }
            if(currentGamepad.dpad_down && !previousGamepad.dpad_down){
                // shift down if not already at lowest. If in FTC dashboard, go to normal
                switch (currentSpeed){
                    case CUSTOM_FTC_DASHBOARD:
                    case FAST:
                        currentSpeed = constants.SPEEDS.NORMAL;
                        break;
                    case NORMAL:
                    case SLOW:
                        currentSpeed = constants.SPEEDS.SLOW;
                }
            }
            // todo make this only work while dashboard is running (if thats possible)
            if(currentGamepad.dpad_left && !previousGamepad.dpad_left){
                // go to custom ftc dashboard speed
                currentSpeed = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
            }

            double speedNow;
            switch (currentSpeed){
                case SLOW:
                    speedNow = constants.SLOW_SPEED;
                    break;
                default: // this errors if doesn't have it idk why
                case NORMAL:
                    speedNow = constants.NORMAL_SPEED;
                    break;
                case FAST:
                    speedNow = constants.FAST_SPEED;
                    break;
                case CUSTOM_FTC_DASHBOARD:
                    speedNow = constants.CUSTOM_FTC_DASHBOARD_SPEED;
                    break;
            }

            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

            double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

            double leftFrontPower  = (stickY + stickX + stickRotation) / maxPower * speedNow;
            double leftBackPower  = (stickY - stickX + stickRotation) / maxPower * speedNow;
            double rightFrontPower  = (stickY - stickX - stickRotation) / maxPower * speedNow;
            double rightBackPower  = (stickY + stickX - stickRotation) / maxPower * speedNow;

            robot.writeToTelemetry("LeftMotorPower", leftFrontPower);
            robot.writeToTelemetry("LeftBackPower", leftBackPower);
            robot.writeToTelemetry("RightFrontPower", rightFrontPower);
            robot.writeToTelemetry("RightBackPower", rightBackPower);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updateTelemetry();

            previousGamepad = currentGamepad;
        }
    }
}
