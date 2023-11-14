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
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    public void runOpMode() {
        robot = new centerStageRobot(this);
        int tmp_deadzoneadjust = 2;
        previousGamepad.copy(currentGamepad);
        waitForStart();

        while(opModeIsActive()){
            currentGamepad.copy(gamepad1);
            updateButtons(currentGamepad);

            double speedNow;
            switch (constants.currentSpeedMode){
                case SLOW:
                    speedNow = constants.SLOW_SPEED;
                    break;
                default: // we really shouldn't need this but it errors if we don't have it
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
            robot.writeToTelemetry("Current Speed Mode", constants.currentSpeedMode);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updateTelemetry();

            previousGamepad.copy(currentGamepad);
        }
    }
    private void updateButtons(Gamepad currentGamepad){
        if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
            constants.currentSpeedMode = constants.SPEEDS.FAST;
        }
        if(currentGamepad.dpad_right && !previousGamepad.dpad_right){
            constants.currentSpeedMode = constants.SPEEDS.NORMAL;
        }
        if(currentGamepad.dpad_down && !previousGamepad.dpad_down){
            constants.currentSpeedMode = constants.SPEEDS.SLOW;
        }
        if(currentGamepad.dpad_left && !previousGamepad.dpad_left){
            // todo make this only work while dashboard is running (if thats possible)
            constants.currentSpeedMode = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
        }
    }
}
