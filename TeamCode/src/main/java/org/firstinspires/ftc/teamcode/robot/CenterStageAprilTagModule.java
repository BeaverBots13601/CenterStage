package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.structures.AprilTagData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class CenterStageAprilTagModule {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ArrayList<AprilTagData> lastData;
    public CenterStageAprilTagModule(OpenCvCamera camera){
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // todo does this angle unit ever matter
                .build(); // todo camera calibration
        //aprilTag.setDecimation(); // todo me

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.setCameraResolution(new Size(cameraWidth, cameraHeight));
        builder.enableLiveView(true); // todo do we need this?
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // todo use this data into AprilTagData
        while(true){
            lastData = new ArrayList();
            for(AprilTagDetection i : aprilTag.getDetections()){
                lastData.add(new AprilTagData(i.id));
            }
        }
    }
}
