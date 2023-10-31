package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
@TeleOp(name="Teleop Controls", group ="CenterStage")
public class centerStageTele extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        centerStageRobot robot = new centerStageRobot(this);
        waitForStart();
        int tmp_deadzoneadjust = 2;

        while(opModeIsActive()){
            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;
            double speed = 0.6;


//            Orientation orientation = robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
//            telemetry.addData("IMU DATA (rads):", orientation);

            // TODO: Learn weird math that definitely makes sense to someone, but not me.
            double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);
            /* Alex suggestion
            double leftFrontPower  = (stickY + stickX + stickRotation) % (maxPower * speed);
            double leftBackPower   = (stickY - stickX + stickRotation) % (maxPower * speed);
            double rightFrontPower = (stickY - stickX - stickRotation) % (maxPower * speed);
            double rightBackPower  = (stickY + stickX - stickRotation) % (maxPower * speed);
             */

            // Ben code
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
