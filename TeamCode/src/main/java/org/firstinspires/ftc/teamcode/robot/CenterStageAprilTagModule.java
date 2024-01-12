package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;
import org.firstinspires.ftc.teamcode.structures.VisionPortalEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CenterStageAprilTagModule {
    private AprilTagProcessor aprilTag;
    private VisionPortalEx visionPortal;
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
                .setLensIntrinsics(1500.35, 1500.35, 961.278, 563.176)
                .build();
        //aprilTag.setDecimation(); // todo me

        VisionPortalEx.Builder builder = new VisionPortalEx.Builder();
        builder.setCamera(cameraNameObject);
        builder.setCameraResolution(new Size(cameraWidth, cameraHeight));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // WARNING: Non-standard function added by us.
        camera = visionPortal.getActiveCameraRaw();
    }

    /**
     * Takes the current camera view and returns information about all visible AprilTags.
     * @return An array of objects, each signifying a detection of an AprilTag and containing data about it.
     */
    public ArrayList<AprilTagData> updateAprilTagData(){
        ArrayList<AprilTagData> data = new ArrayList<>();
        for(AprilTagDetection i : aprilTag.getDetections()){
            data.add(new AprilTagData(i.id, i.ftcPose.y, i.hamming, i.corners, i.center));
        }

        tagData = data;
        return data;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public ArrayList<AprilTagData> getAprilTagData(){
        return tagData;
    }
}
