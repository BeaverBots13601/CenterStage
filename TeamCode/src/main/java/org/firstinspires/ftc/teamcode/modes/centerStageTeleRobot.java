package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;

@TeleOp(name="TeleOp Controls (Robot)", group ="CenterStage")
public class centerStageTeleRobot extends LinearOpMode {
    private centerStageRobot robot;
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private boolean servoClosed = true;
    private boolean liftDown = true;

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
            // todo make this only work while dashboard is running (if that's possible)
            constants.currentSpeedMode = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
        }
        if(currentGamepad.options && !previousGamepad.options){
            // pal
            robot.getPALServo().setDirection(Servo.Direction.REVERSE);
            robot.getPALServo().setPosition(0);
        }
        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            // servo on/off
            if(servoClosed){
                // gets moved away from claw
                robot.getKnockerServo().setPosition(0.67);
                servoClosed = false;
            } else {
                // gets moved to claw
                robot.getKnockerServo().setPosition(1.0);
                servoClosed = true;
            }
        }
        // Lift/Lower grapple lift
        if(currentGamepad.left_bumper && !previousGamepad.left_bumper){
            // fixme this code doesn't really work how we think it does. see above knocker servo code for 'correct' implementation
            if(liftDown) {
                robot.getGrappleServo().setDirection(Servo.Direction.FORWARD);
                robot.getGrappleServo().setPosition(.45);
                robot.writeToTelemetry("Grapple Lift Up", true);
                liftDown = false;
            } else {
                robot.getGrappleServo().setDirection(Servo.Direction.REVERSE);
                robot.getGrappleServo().setPosition(0);
                robot.writeToTelemetry("Grapple Lift Up", false);
                liftDown = true;
            }
        }
        // Make robot pull itself up
        if(currentGamepad.share && !previousGamepad.share){
            DcMotorEx motor = robot.getGrappleMotor();
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fixme don't think this idea works
            motor.setPower(0.95);
            sleep(8000);
            motor.setPower(0.5);
            sleep(constants.GRAPPLE_WAIT_TIME_MS);
            motor.setMotorDisable();
        }
    }
}
