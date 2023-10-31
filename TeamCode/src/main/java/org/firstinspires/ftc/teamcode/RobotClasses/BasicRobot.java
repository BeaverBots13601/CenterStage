package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Constants.MOTOR_TICKS_PER_REV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClasses.BasicRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Owns and provides the basic IMU and Motor objects. Does not handle driving, but does handle configuration.
 */
public class BasicRobot extends LinearOpMode {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private Map<String, DcMotorEx> driveMotors;
    private double wheelDiameter;
    //private Map<String, DcMotorEx> launcherMotors;
    private IMU imu;

    public BasicRobot(HardwareMap hardwareMap, Telemetry telemetry, double wheelDiameter) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.wheelDiameter = wheelDiameter;
        // TODO: More info abt this. FTCDashboard thing?
        telemetry.setMsTransmissionInterval(25);
        createDriveMotors();
        //imu = createImu();
        initBulkReads();

        /*
        Map<String, DcMotorEx> tmp;
        try {
            tmp = createLauncherMotors();
        } catch(Exception e){
            telemetry.addLine("Launcher motors initialized failed. Proceeding as if functionality unavailable.");
            tmp = new HashMap<>();
        }
        launcherMotors = tmp;*/
    }


    private DcMotorEx createDefaultMotor(String motorName) {
        DcMotorEx motor = this.hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motorName.toLowerCase().contains("left")) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }

    private void createDriveMotors()  {
        this.driveMotors = new HashMap<>();
        for (String driveMotorName : Constants.DRIVE_MOTOR_NAMES) {
            DcMotorEx driveMotor = createDefaultMotor(driveMotorName);
            this.driveMotors.put(driveMotorName, driveMotor);
        }
    }

    private IMU createImu() {
        // This creates a warning but that's ok its for backwards compat and doesn't break anything
        BNO055IMUNew.Parameters imuParameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        //imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        boolean worked = imu.initialize(imuParameters);
        telemetry.addData("IMU Initialized Goodly?", worked); telemetry.update();

        return imu;
    }

    public void driveInches(double inches, double power) {
        int target = (int)(inches * (MOTOR_TICKS_PER_REV / (this.wheelDiameter * Math.PI)));
        ElapsedTime     runtime = new ElapsedTime();
        //set all encoders to 0
//        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
//            DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            // Send telemetry message to indicate successful Encoder reset
//            telemetry.addData("Running to", " %7d", target);
//            telemetry.addData("Starting at", "%s, %7d",
//                    motorName,
//                    motor.getCurrentPosition());
//            telemetry.addLine();
//        }

        //turn on all motors
        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
            DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
            motor.setTargetPosition(target);
            telemetry.addLine("Motors turned on");
        }
        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
            DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
            DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
            motor.setPower(Math.abs(power));
        }

        runtime.reset();
        telemetry.addData("Is driving? ", "%b", this.isDriving());
        telemetry.addData("is active?", "%b", opModeIsActive());

        while (this.isDriving()) {
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("driving? ", "%b", this.isDriving());
            for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
                DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
                telemetry.addData("Running to", " %7d", target);
                telemetry.addData("Currently  at",  "%s, %7d",
                        motorName,
                        motor.getCurrentPosition());
                telemetry.addData("Is driving? ", "%b", motor.isBusy());
                telemetry.addData("is active?", "%b", opModeIsActive());
                telemetry.addLine();
            }
        }
        telemetry.addLine();
        telemetry.addLine();

        //turn off the motors TOOD: Is Jerky
        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
            DcMotor motor = this.hardwareMap.get(DcMotorEx.class, motorName);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Motors turned off");
            telemetry.addData("Currently  at",  "%s, %7d",
                    motorName,
                    motor.getCurrentPosition());
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.driveInches(10, .5);
    }

    private boolean isDriving() {
//        for (String motorName : Constants.DRIVE_MOTOR_NAMES) {
//            telemetry.addData("Is Busy: ", "%s, %b", motorName, this.hardwareMap.get(DcMotorEx.class, motorName).isBusy());
//            if (this.hardwareMap.get(DcMotorEx.class, motorName).isBusy()) {
//                return true;
//            }
//        }
        return this.getLeftBack().isBusy() || this.getRightBack().isBusy() || this.getLeftFront().isBusy() || this.getRightFront().isBusy();
    }



    // TODO: How does this work
    private void initBulkReads() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public DcMotorEx getLeftFront() {
        return this.driveMotors.get("leftFront");
    }
    public DcMotorEx getLeftBack() {
        return this.driveMotors.get("leftBack");
    }
    public DcMotorEx getRightFront() {
        return this.driveMotors.get("rightFront");
    }
    public DcMotorEx getRightBack() {
        return this.driveMotors.get("rightBack");
    }

    public IMU getImu() {
        return imu;
    }


}
