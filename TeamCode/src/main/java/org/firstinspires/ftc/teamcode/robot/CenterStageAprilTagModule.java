package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CenterStageAprilTagModule extends OpenCvPipeline {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ArrayList<AprilTagData> tagData;
    private OpenCvCamera camera;

    /**
     * Handles the scanning and recognition of AprilTags, in addition to the setup process for the camera.
     * @param cameraNameObject The WebcamName object of the camera.
     */
    public CenterStageAprilTagModule(WebcamName cameraNameObject, int cameraWidth, int cameraHeight, OpenCvCameraRotation cameraOrientation){
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // ang unit doesn't matter
                .build(); // todo camera calibration
        //aprilTag.setDecimation(); // todo me

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(cameraNameObject);
        builder.setCameraResolution(new Size(cameraWidth, cameraHeight));
        builder.enableLiveView(false);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        camera = OpenCvCameraFactory.getInstance().createWebcam(cameraNameObject);
        // This sets up the camera n stuff. Basically just does settings
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
                camera.startStreaming(cameraWidth, cameraHeight, cameraOrientation);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }

    public Mat processFrame(Mat in){
        tagData = updateAprilTagData();

        for(AprilTagData i : tagData){
            // todo draw boxes around tags
        }

        return in;
    }

    /**
     * Takes the current camera view and returns information about all visible AprilTags.
     * @return An array of objects, each signifying a detection of an AprilTag and containing data about it.
     */
    private ArrayList<AprilTagData> updateAprilTagData(){
        ArrayList<AprilTagData> data = new ArrayList<>();
        for(AprilTagDetection i : aprilTag.getDetections()){
            data.add(new AprilTagData(i.metadata.id, i.ftcPose.y, i.hamming, i.corners, i.center));
        }

        return data;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public ArrayList<AprilTagData> getAprilTagData(){
        return tagData;
    }
}
