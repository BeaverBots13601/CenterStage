package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.structures.Pose;

@TeleOp(name="Teleop Controls (Robot)", group ="CenterStage")
public class centerStageTeleRobot extends LinearOpMode {
    private centerStageRobot robot;

    public void runOpMode() throws InterruptedException {
        robot = new centerStageRobot(this);
        int tmp_deadzoneadjust = 2;
        double speed = 0.65;

        waitForStart();


        while(opModeIsActive()){
            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

            double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

            double leftFrontPower  = (stickY + stickX + stickRotation) / maxPower * speed;
            double leftBackPower  = (stickY - stickX + stickRotation) / maxPower * speed;
            double rightFrontPower  = (stickY - stickX - stickRotation) / maxPower * speed;
            double rightBackPower  = (stickY + stickX - stickRotation) / maxPower * speed;

            telemetry.addData("LeftMotorPower", leftFrontPower);
            telemetry.addData("LeftBackPower", leftBackPower);
            telemetry.addData("RightFrontPower", rightFrontPower);
            telemetry.addData("RightBackPower", rightBackPower);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.update();
        }
    }
}
