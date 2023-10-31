package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;

@Disabled
public abstract class baseRobot extends LinearOpMode {

    final TreeMap<String, DcMotorEx> driveMotors;
    private final double wheelDiameter;
    private final double robotDiameter;

    public baseRobot(double wheelDiameter, double robotDiameter) {
        this.driveMotors = new TreeMap<>();
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
        for (String driveMotorName : constants.DRIVE_MOTOR_NAMES) {
            DcMotorEx driveMotor = createDefaultMotor(driveMotorName);
            this.driveMotors.put(driveMotorName, driveMotor);
        }
    }

    public void setDriveMotors(double[] powers, DcMotor.RunMode mode) {
        int i = 0;
        for (Map.Entry<String, DcMotorEx> motor : this.driveMotors.entrySet()) {
            motor.getValue().setMode(mode);
            motor.getValue().setPower(powers[i]);
            i++;
        }
    }

    public void stopDrive() {
        double[] powers = new double[this.driveMotors.size()];
        Arrays.fill(powers, 0.0);
        setDriveMotors(powers, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isDriving() {
        for (Map.Entry<String, DcMotorEx> motor : this.driveMotors.entrySet()) {
            if (motor.getValue().isBusy()) {
                return true;
            }
        }
        return false;
    }

    double inchesToEncoder(double inches) {
        return inches * (constants.ENCODER_TICKS / (this.wheelDiameter * Math.PI));
    }

    public void driveEncoded(int[] ticks, double[] powers) {
        int i = 0;
        for (Map.Entry<String, DcMotorEx> motor : this.driveMotors.entrySet()) {
            motor.getValue().setTargetPosition(ticks[i]);
            i++;
        }

        this.setDriveMotors(powers, DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && this.isDriving()) {
            i = 0;
            for (Map.Entry<String, DcMotorEx> motor : this.driveMotors.entrySet()) {
                telemetry.addData("Running to", " %7d", ticks[i]);
                telemetry.addData("Currently at", "%s at %7d",
                        motor.getKey(),
                        motor.getValue().getCurrentPosition());
                i++;
            }
            telemetry.update();
        }

        this.stopDrive();
    }

    public void driveInches(double inches, double power) {
        int[] target = new int[this.driveMotors.size()];
        double[] powers = new double[this.driveMotors.size()];
        Arrays.fill(target, (int) this.inchesToEncoder(inches));
        Arrays.fill(powers, power);

        driveEncoded(target, powers);
    }

    public void turnDegrees(int degrees, double power) {
        int[] target = new int[this.driveMotors.size()];
        double[] powers = new double[] {power, power, -power, -power};
        Arrays.fill(target, (int) this.inchesToEncoder(Math.toRadians(degrees) * this.robotDiameter));

        driveEncoded(target, powers);
    }


}