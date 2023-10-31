package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@Disabled
public abstract class baseRobot extends LinearOpMode {

    DcMotorEx[] driveMotors;
    private final double wheelDiameter;
    private final double robotDiameter;

    public baseRobot(double wheelDiameter, double robotDiameter) {
        this.driveMotors = new DcMotorEx[constants.driveMotorName.values().length];
        this.telemetry.setMsTransmissionInterval(constants.TELEMETRY_MS_TRANSMISSION_INTERVAL);
        createDriveMotors();

        this.wheelDiameter= wheelDiameter;
        this.robotDiameter = robotDiameter;

        this.telemetry.addData(">", "Hardware Initialized");
        this.telemetry.update();
    }

    private DcMotorEx createDefaultMotor(String motorName) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motorName.toLowerCase().contains("left")) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }

    private void createDriveMotors() {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            DcMotorEx driveMotor = createDefaultMotor(driveMotorName.name());
            this.driveMotors[driveMotorName.ordinal()] = driveMotor;
        }
    }

    public void setDriveMotors(double[] powers, DcMotor.RunMode mode) {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setMode(mode);
            this.driveMotors[driveMotorName.ordinal()].setPower(powers[driveMotorName.ordinal()]);
        }
    }

    public void stopDrive() {
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(powers, 0.0);
        setDriveMotors(powers, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isDriving() {
        for (DcMotorEx motor : this.driveMotors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    double inchesToEncoder(double inches) {
        return inches * (constants.ENCODER_TICKS / (this.wheelDiameter * Math.PI));
    }

    public void driveEncoded(int[] ticks, double[] powers) {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setTargetPosition(ticks[driveMotorName.ordinal()]);
        }

        this.setDriveMotors(powers, DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && this.isDriving()) {
            for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
                telemetry.addData("Running to", " %7d", ticks[driveMotorName.ordinal()]);
                telemetry.addData("Currently at", "%s at %7d",
                        driveMotorName.name(),
                        this.driveMotors[driveMotorName.ordinal()].getCurrentPosition());
            }
            telemetry.update();
        }

        this.stopDrive();
    }

    public void driveInches(double inches, double power) {
        int[] target = new int[this.driveMotors.length];
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(target, (int) this.inchesToEncoder(inches));
        Arrays.fill(powers, power);

        driveEncoded(target, powers);
    }

    public void turnDegrees(int degrees, double power) {
        int[] target = new int[this.driveMotors.length];
        double[] powers = new double[] {power, power, -power, -power};
        Arrays.fill(target, (int) this.inchesToEncoder(Math.toRadians(degrees) * this.robotDiameter));

        driveEncoded(target, powers);
    }


}