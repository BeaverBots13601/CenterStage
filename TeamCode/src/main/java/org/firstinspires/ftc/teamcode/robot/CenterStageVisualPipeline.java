package org.firstinspires.ftc.teamcode.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStageVisualPipeline extends OpenCvPipeline {
    //    Changes how "zoomed-in" the camera sees
    static final double SQUARE_SIZE_PX = 75;
    static final Rect ROI = new Rect(
            new Point(constants.CAMERA_WIDTH / 2.0 - SQUARE_SIZE_PX / 2.0,
                    constants.CAMERA_HEIGHT / 2.0 - SQUARE_SIZE_PX / 2.0),
            new Point(constants.CAMERA_WIDTH / 2.0 + SQUARE_SIZE_PX / 2.0,
                    constants.CAMERA_HEIGHT / 2.0 + SQUARE_SIZE_PX / 2.0)
    );
    static final double HUE_DIFF = 15;
    static final double SAT_DIFF = 205;
    static final double VAL_DIFF = 185;

    //    The hue value clips at 180 and turns to 0 at red, so we need two ranges.
//    Red color range below 180
    static final Scalar redHSVLow1 = new Scalar(180 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar redHSVHigh1 = new Scalar(180, 255, 255);
    //    Red color range above 0
    static final Scalar redHSVLow2 = new Scalar(0, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar redHSVHigh2 = new Scalar(0 + HUE_DIFF, 255, 255);
    //    Green color range
    static final Scalar greenHSVLow = new Scalar(60 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar greenHSVHigh = new Scalar(60 + HUE_DIFF, 255, 255);
    //    Blue color range
    static final Scalar blueHSVLow = new Scalar(120 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar blueHSVHigh = new Scalar(120 + HUE_DIFF, 255, 255);

    private final Mat hsv = new Mat();
    private final Mat grey = new Mat();
    private Mat center;

    public enum SleeveColor {
        RED,
        GREEN,
        BLUE
    }
    private SleeveColor lastResult;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
        center = grey.submat(ROI);
        double redPercentage = Core.sumElems(center).val[0] / ROI.area() / 255;

        Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
        center = grey.submat(ROI);
        redPercentage += redPercentage + Core.sumElems(center).val[0] / ROI.area() / 255;
        redPercentage /= 2.0;

        Core.inRange(hsv, greenHSVLow, greenHSVHigh, grey);
        center = grey.submat(ROI);
        double greenPercentage = Core.sumElems(center).val[0] / ROI.area() / 255;

        Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
        center = grey.submat(ROI);
        double bluePercentage = Core.sumElems(center).val[0] / ROI.area() / 255;

        double max = Math.max(redPercentage, greenPercentage);
        max = Math.max(max, bluePercentage);

        Scalar color;
        if (max == redPercentage) {
            lastResult = SleeveColor.RED;
//            Mix both ranges in the final display?
            color = new Scalar(255, 0, 0);
        } else if (max == greenPercentage) {
            lastResult = SleeveColor.GREEN;
            color = new Scalar(0, 255, 0);
        } else {
            lastResult = SleeveColor.BLUE;
            color = new Scalar(0, 0, 255);
        }

        Imgproc.rectangle(input, ROI, color);
        return input;
    }

    public SleeveColor getLastResult() {
        return lastResult;
    }
}
