package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.robot.constants;
import org.firstinspires.ftc.teamcode.structures.Pose;

@TeleOp(name="TeleOp Controls (Field)", group ="CenterStage")
public class centerStageTeleField extends LinearOpMode {
    private constants.OrientationMode orientationMode = constants.OrientationMode.FIELD;
    private centerStageRobot robot;
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private boolean servoOpen = false;

    public void runOpMode() {
        robot = new centerStageRobot(this);
        double referenceAngle = robot.getImuAngle();
        int tmp_deadzoneadjust = 2;
        previousGamepad.copy(currentGamepad);

        waitForStart();

        while(opModeIsActive()){
            currentGamepad.copy(gamepad1);
            updateButtons(currentGamepad, previousGamepad);

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

            double directionRotation = -Pose.normalizeAngle(robot.getImuAngle() - referenceAngle);

            Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
            double rotatedStickX = rotatedPosition.getX();
            double rotatedStickY = rotatedPosition.getY();
            Orientation orientation = robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            robot.writeToTelemetry("IMU DATA (rads):", orientation);

            double maxPower = Math.max(Math.abs(rotatedStickY) + Math.abs(rotatedStickX) + Math.abs(stickRotation), 1);

            double leftFrontPower  = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow;
            double leftBackPower   = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow;
            double rightFrontPower = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow;
            double rightBackPower  = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow;

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
    private void updateButtons(Gamepad currentGamepad, Gamepad previousGamepad){
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
        if(currentGamepad.options && !previousGamepad.options){
            // pal
            robot.getPALServo().setPosition((constants.PAL_ROTATION_DEGREES / 300)); // # of degrees, out of 300 DOM
        }
        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            // servo on/off
            if(servoOpen){
                robot.getKnockerServo().setPosition(-(constants.KNOCKER_ROTATION_DEGREES / 300));
            } else {
                robot.getKnockerServo().setPosition((constants.KNOCKER_ROTATION_DEGREES / 300));
            }
        }
    }
}
