package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

public abstract class baseRobot {

    DcMotorEx[] driveMotors;
    private LinearOpMode opMode;
    private final double wheelDiameter;
    private final double robotDiameter;
    private final IMU imu;

    public baseRobot(LinearOpMode opmode, double wheelDiameter, double robotDiameter) {
        super();
        this.opMode = opmode;
        this.driveMotors = new DcMotorEx[constants.driveMotorName.values().length];
        this.opMode.telemetry.setMsTransmissionInterval(constants.TELEMETRY_MS_TRANSMISSION_INTERVAL);
        createDriveMotors();

        this.wheelDiameter= wheelDiameter;
        this.robotDiameter = robotDiameter;
        this.imu = createImu();

        this.opMode.telemetry.addData(">", "Hardware Initialized");
        this.opMode.telemetry.update();
    }

    private DcMotorEx createDefaultMotor(String motorName) {
        DcMotorEx motor = this.opMode.hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            this.opMode.telemetry.addData("set ", "%s: %f", driveMotorName.name(), powers[driveMotorName.ordinal()]);
        }
    }

    public void stopDrive() {
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(powers, 0.0);
        setDriveMotors(powers, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        return (inches * constants.ENCODER_TICKS / (this.wheelDiameter * Math.PI));
    }

    /**
     * Drive X number of encoder ticks
     * @param powers Array of powers in order of leftFront, leftBack, rightFront, rightBack
     */
    public void driveEncoded(int[] ticks, double[] powers) {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setTargetPosition(ticks[driveMotorName.ordinal()]);
        }

        this.setDriveMotors(powers, DcMotor.RunMode.RUN_TO_POSITION);

        while (this.opMode.opModeIsActive() && this.isDriving()) {
            for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
                this.opMode.telemetry.addData("Running to", " %7d", ticks[driveMotorName.ordinal()]);
                this.opMode.telemetry.addData("Currently at", "%s at %7d",
                        driveMotorName.name(),
                        this.driveMotors[driveMotorName.ordinal()].getCurrentPosition());
            }
            this.opMode.telemetry.update();
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

    /**
     * turns degrees
     * @param degrees Degs to turn. Positive is to the right, negative to the left
     * @param power
     */
    public void turnDegrees(int degrees, double power) {
        int targetInches = (int) this.inchesToEncoder(Math.toRadians(degrees) * this.robotDiameter);
        int[] target = new int[] {targetInches, targetInches, -targetInches, -targetInches};
        double[] powers = new double[] {power, power, -power, -power};

        driveEncoded(target, powers);
    }

    private IMU createImu() {
        // This creates a warning but that's ok its for backwards compat and doesn't break anything
        BNO055IMUNew.Parameters imuParameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        //imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        IMU imu = opMode.hardwareMap.get(IMU.class, "imu");
        boolean worked = imu.initialize(imuParameters);
        opMode.telemetry.addData("IMU Initialized Goodly?", worked); opMode.telemetry.update();

        return imu;
    }

    public IMU getImu() {
        return imu;
    }

    /**
     * Enables bulk reads which allow faster hardware call times.
     * Set to auto currently but if speed becomes an issue this can be manually configured.
     */
    private void initBulkReads() {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}