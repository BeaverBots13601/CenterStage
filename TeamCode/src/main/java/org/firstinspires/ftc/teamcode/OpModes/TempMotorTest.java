package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TempMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx borkedMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx testMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        while(opModeIsActive()){
            borkedMotor.setPower(0.4);
            testMotor.setPower(0.4);
            sleep(2500);
            borkedMotor.setPower(0);
            testMotor.setPower(0);
            sleep(5000);
        }
    }
}
