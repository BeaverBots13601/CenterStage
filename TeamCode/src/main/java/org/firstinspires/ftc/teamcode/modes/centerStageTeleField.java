package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public void runOpMode() {
        robot = new centerStageRobot(this);
        double referenceAngle = robot.getImuAngle();
        int tmp_deadzoneadjust = 2;
        double speed = 0.65;

        waitForStart();

        while(opModeIsActive()){
            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

            double directionRotation =  -Pose.normalizeAngle(robot.getImuAngle() - referenceAngle);

            Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
            double rotatedStickX = rotatedPosition.getX();
            double rotatedStickY = rotatedPosition.getY();
            Orientation orientation = robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            robot.writeToTelemetry("IMU DATA (rads):", orientation);

            double maxPower = Math.max(Math.abs(rotatedStickY) + Math.abs(rotatedStickX) + Math.abs(stickRotation), 1);

            double leftFrontPower  = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speed;
            double leftBackPower  = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speed;
            double rightFrontPower  = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speed;
            double rightBackPower  = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speed;

            robot.writeToTelemetry("LeftMotorPower", leftFrontPower);
            robot.writeToTelemetry("LeftBackPower", leftBackPower);
            robot.writeToTelemetry("RightFrontPower", rightFrontPower);
            robot.writeToTelemetry("RightBackPower", rightBackPower);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updateTelemetry();
        }
    }
}
