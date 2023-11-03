package org.firstinspires.ftc.teamcode.modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.centerStageRobot;
import org.firstinspires.ftc.teamcode.structures.Pose;

@TeleOp(name="Teleop Controls (Field)", group ="CenterStage")
public class centerStageTeleField extends LinearOpMode {
    private enum OrientationMode {
        FIELD,
        ROBOT
    }
    private OrientationMode orientationMode = OrientationMode.FIELD;
    private double referenceAngle;
    private centerStageRobot robot;

    public void runOpMode() throws InterruptedException {
        robot = new centerStageRobot(this);
        referenceAngle = getImuAngle();
        waitForStart();
        int tmp_deadzoneadjust = 2;

        while(opModeIsActive()){
            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;
            double speed = 0.65;


            double directionRotation = 0;
            if (orientationMode == OrientationMode.FIELD) {
                directionRotation = -normalizeAngle(getImuAngle() - referenceAngle);
            }

            Pose rotatedPosition = rotatePosition(stickX, stickY, directionRotation);
            double rotatedStickX = rotatedPosition.getX();
            double rotatedStickY = rotatedPosition.getY();
//            Orientation orientation = robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
//            telemetry.addData("IMU DATA (rads):", orientation);

            // TODO: Learn weird math that definitely makes sense to someone, but not me.
            double maxPower = Math.max(Math.abs(rotatedStickY) + Math.abs(rotatedStickX) + Math.abs(stickRotation), 1);
            /* Alex suggestion
            double leftFrontPower  = (stickY + stickX + stickRotation) % (maxPower * speed);
            double leftBackPower   = (stickY - stickX + stickRotation) % (maxPower * speed);
            double rightFrontPower = (stickY - stickX - stickRotation) % (maxPower * speed);
            double rightBackPower  = (stickY + stickX - stickRotation) % (maxPower * speed);
             */

            // Ben code
            double leftFrontPower  = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speed;
            double leftBackPower  = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speed;
            double rightFrontPower  = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speed;
            double rightBackPower  = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speed;

            telemetry.addData("LeftMotorPower", leftFrontPower);
            telemetry.addData("LeftBackPower", leftBackPower);
            telemetry.addData("RightFrontPower", rightFrontPower);
            telemetry.addData("RightBackPower", rightBackPower);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.update();
        }
    }

    /**
     * @param x x value of the point
     * @param y y value of the point
     * @param angle angle change of the point
     * @return rotation of point (x, y) around angle
     */
    private static Pose rotatePosition(double x, double y, double angle) {
        return new Pose(x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle), angle);
    }

    /**
     * @return double imu angle around the vertical axis (rotation).
     */
    private double getImuAngle() {
        return robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    /**
     * @return double angle within the range [-π, π]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
