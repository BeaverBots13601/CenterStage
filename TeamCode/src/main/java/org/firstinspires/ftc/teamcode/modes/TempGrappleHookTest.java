package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Temp Grapple Test")
public class TempGrappleHookTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "rightBack");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        //doIntervalPower(motor, 0.8, 0.05);
        motor.setPower(0.95);
        sleep(6000);
        motor.setPower(0.5);
        sleep(30000);
        /*motor.setPower(0.35);
        sleep(10000);
        motor.setPower(0.2);
        sleep(5000);*/
        motor.setPower(0);
    }

    private void doIntervalPower(DcMotorEx motor, double targetPower, double powerStep){
        double powerDifference = targetPower - motor.getPower();
        while (motor.getPower() != targetPower) {
            if (powerDifference > powerStep) {
                motor.setPower(motor.getPower() + powerStep);
            } else if (powerDifference < -powerStep) {
                motor.setPower(motor.getPower() - powerStep);
            } else {
                motor.setPower(targetPower);
            }
        }
    }
}
