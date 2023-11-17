package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

public class centerStageRobot extends baseRobot {
    private Servo knockerServo;
    private Servo PALServo;

    private DcMotorEx grappleMotor;

    private OpenCvCamera camera;

    public centerStageRobot(LinearOpMode opmode) {
        super(opmode, 3.5, 13.75);
        this.knockerServo = setUpServo(constants.KNOCKER_SERVO_NAME);
        //this.PALServo = setUpServo(constants.PAL_SERVO_NAME);

        //this.grappleMotor = createDefaultMotor("grappleMotor");

        // todo: I think this camera can go up to 1920x1080 & 60fps. Not sure, test maybe?
        this.camera = setUpCamera(constants.CAMERA_NAME, constants.CAMERA_WIDTH, constants.CAMERA_HEIGHT);
        dashboard.startCameraStream(camera, 60);
    }

    public void driveStrafe(double inches, double power) {
        int ticks = (int) this.inchesToEncoder(inches);
        int[] target = new int[] {ticks, -ticks, -ticks, ticks};
        double[] powers = new double[] {power, -power, -power, power};

        driveEncoded(target, powers);
    }

    public AprilTagData[] identifyTags() {
        return new AprilTagData[] {};
    }


    private Servo setUpServo(String servoName){
        Servo servo = opMode.hardwareMap.get(Servo.class, servoName);
        return servo;
    }

    public Servo getKnockerServo(){
        // servo specs: GoBilda 2000 Series Dual Mode Servo (25-2, Torque)
        // SKU: 2000-0025-0002
        return knockerServo;
    }

    public Servo getPALServo() {
        return PALServo;
    }

    public DcMotorEx getGrappleMotor(){
        return grappleMotor;
    }

    private OpenCvCamera setUpCamera(String cameraName, int cameraWidth, int cameraHeight){
        // todo, What's the point of this intermediate step?
        WebcamName cameraNameThing = opMode.hardwareMap.get(WebcamName.class, cameraName);
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraNameThing);
        // This sets up the camera n stuff. Basically just does settings
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        return webcam;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }
}
