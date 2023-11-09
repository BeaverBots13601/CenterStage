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
    private Gamepad previousGamepad;
    private constants.SPEEDS currentSpeed = constants.SPEEDS.NORMAL;

    public void runOpMode() {
        robot = new centerStageRobot(this);
        double referenceAngle = robot.getImuAngle();
        int tmp_deadzoneadjust = 2;

        waitForStart();
        previousGamepad = gamepad1;

        while(opModeIsActive()){
            Gamepad currentGamepad = gamepad1;
            telemetry.addData("DPAD UP", currentGamepad.dpad_up);
            telemetry.addData("PREV DPAD UP", previousGamepad.dpad_up);
            telemetry.update();

            if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
                telemetry.addLine("Pressed dpad up");
                telemetry.update();
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
                telemetry.addLine("Pressed dpad down");
                telemetry.update();
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
                telemetry.addLine("Pressed dpad left");
                telemetry.update();
                // go to custom ftc dashboard speed
                currentSpeed = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
            }

            double speedNow;
            switch (currentSpeed){
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
            robot.writeToTelemetry("Current Speed Mode", currentSpeed);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updateTelemetry();

            previousGamepad = currentGamepad;
        }
    }
}
