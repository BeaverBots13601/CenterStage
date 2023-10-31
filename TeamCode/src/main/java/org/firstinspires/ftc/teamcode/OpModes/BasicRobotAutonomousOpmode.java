package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.LegacyClasses.AnglePID;
import org.firstinspires.ftc.teamcode.LegacyClasses.EncoderPositions;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LegacyClasses.Pose;
import org.firstinspires.ftc.teamcode.LegacyClasses.PID;
import org.firstinspires.ftc.teamcode.RobotClasses.BasicRobot;

@Autonomous(name="Basic Autonomous Mode (No Claw)")
public class BasicRobotAutonomousOpmode extends LinearOpMode {
    private BasicRobot robot;
    private EncoderPositions previousEncoderPositions;
    private EncoderPositions currentEncoderPositions;
    private Pose targetPosition;
    private Pose position = new Pose();

    private PID xControl;
    private PID yControl;
    private AnglePID angleControl;
    private double referenceAngle;

    @Override
    public void runOpMode() {
        robot = new BasicRobot(hardwareMap, telemetry, 3);
        currentEncoderPositions = createCurrentEncoderPositions();
        referenceAngle = getImuAngle();
        waitForStart();
        while(opModeIsActive()) {

            // todo: are the values in pose measured in inches?
            // todo this needs testing 100000%
            driveToPosition(new Pose(1, 0, 0));
            driveToPosition(new Pose(0, 11, 0));
        }
    }

    /**
     * @return EncoderPositions with current encoder values of the drive motors.
     */
    private EncoderPositions createCurrentEncoderPositions() {
        return new EncoderPositions(
                robot.getLeftFront().getCurrentPosition(),
                robot.getLeftBack().getCurrentPosition(),
                robot.getRightFront().getCurrentPosition(),
                robot.getRightBack().getCurrentPosition()
        );
    }

    public void driveToPosition(Pose positionDif) {
        changeTargetPosition(positionDif);
        while (!areTargetsReached() && opModeIsActive()) {
            update();
            moveToTarget();

            telemetry.addData("Current Position", position);
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }

    private void changeTargetPosition(Pose positionDif) {
        xControl = new PID(Constants.X_KP, Constants.X_KI, Constants.X_KD);
        yControl = new PID(Constants.KP, Constants.KI, Constants.KD);
        angleControl = new AnglePID(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        Pose currentPosition = position;
        targetPosition = new Pose(
                currentPosition.getX() + positionDif.getX(),
                currentPosition.getY() + positionDif.getY(),
                normalizeAngle(currentPosition.getAngle() + Math.toRadians(-positionDif.getAngle()))
        );
    }

    public boolean isTargetXReached() {
        return Math.abs(targetPosition.getX() - position.getX()) < Constants.PID_INCHES_TOLERANCE;
    }
    public boolean isTargetYReached() {
        return Math.abs(targetPosition.getY() - position.getY()) < Constants.PID_INCHES_TOLERANCE;
    }
    public boolean isTargetAngleReached() {
        return Math.abs(targetPosition.getAngle() - position.getAngle()) < Constants.PID_ANGLE_TOLERANCE;
    }
    public boolean areTargetsReached() {
        return isTargetXReached()
                && isTargetYReached()
                && isTargetAngleReached();
    }

    private void moveToTarget() {
        Pose currentPosition = position;
        double xPid = xControl.calculate(currentPosition.getX(), targetPosition.getX());
        double yPid = yControl.calculate(currentPosition.getY(), targetPosition.getY());
        double anglePid = angleControl.calculate(currentPosition.getAngle(), targetPosition.getAngle());

        // todo learn weird math
        double driveMax = Math.max(Math.abs(yPid) + Math.abs(xPid) + Math.abs(anglePid), 1);
        double leftFrontPower = (yPid + xPid - anglePid) / driveMax * Constants.AUTONOMOUS_DRIVE_SPEED;
        double leftBackPower = (yPid - xPid - anglePid) / driveMax * Constants.AUTONOMOUS_DRIVE_SPEED;
        double rightFrontPower = (yPid - xPid + anglePid) / driveMax * Constants.AUTONOMOUS_DRIVE_SPEED;
        double rightBackPower = (yPid + xPid + anglePid) / driveMax * Constants.AUTONOMOUS_DRIVE_SPEED;

        setLeftFrontPower(leftFrontPower, Constants.AUTO_POWER_STEP);
        setLeftBackPower(leftBackPower, Constants.AUTO_POWER_STEP);
        setRightFrontPower(rightFrontPower, Constants.AUTO_POWER_STEP);
        setRightBackPower(rightBackPower, Constants.AUTO_POWER_STEP);
    }

    private void update() {
        previousEncoderPositions = currentEncoderPositions;
        currentEncoderPositions = createCurrentEncoderPositions();
        double leftFrontDif = currentEncoderPositions.getLeftFront() - previousEncoderPositions.getLeftFront();
        double leftBackDif = currentEncoderPositions.getLeftBack() - previousEncoderPositions.getLeftBack();
        double rightFrontDif = currentEncoderPositions.getRightFront() - previousEncoderPositions.getRightFront();
        double rightBackDif = currentEncoderPositions.getRightBack() - previousEncoderPositions.getRightBack();

        double xDif = (leftFrontDif + rightBackDif - leftBackDif - rightFrontDif) / 4.0 / Constants.TICKS_PER_INCH/* * Constants.LATERAL_WHEEL_SLIP_POSITION_ADJUST*/;
        double yDif = (leftFrontDif + leftBackDif + rightFrontDif + rightBackDif) / 4.0 / Constants.TICKS_PER_INCH/* * Constants.WHEEL_SLIP_POSITION_ADJUST*/;
        double angle = normalizeAngle(getImuAngle() - referenceAngle);


        position = new Pose(position.getX() + xDif, position.getY() + yDif, angle);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    private void stopMovement() {
        setRightFrontPower(0, 2);
        setRightBackPower(0, 2);
        setLeftFrontPower(0, 2);
        setLeftBackPower(0, 2);
    }

    public void setStepPower(DcMotorEx motor, double targetPower, double powerStep) {
        double powerDifference = targetPower - motor.getPower();
        if (powerDifference > powerStep) {
            motor.setPower(motor.getPower() + powerStep);
        } else if (powerDifference < -powerStep) {
            motor.setPower(motor.getPower() - powerStep);
        } else {
            motor.setPower(targetPower);
        }
    }

    public void setLeftFrontPower(double power, double powerStep) {
        setStepPower(robot.getLeftFront(), power, powerStep);
    }
    public void setLeftBackPower(double power, double powerStep) {
        setStepPower(robot.getLeftBack(), power, powerStep);
    }
    public void setRightFrontPower(double power, double powerStep) {
        setStepPower(robot.getRightFront(), power, powerStep);
    }
    public void setRightBackPower(double power, double powerStep) {
        setStepPower(robot.getRightBack(), power, powerStep);
    }

    /**
     * @return double imu angle around the vertical axis (rotation).
     */
    private double getImuAngle() {
        return robot.getImu().getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }
}