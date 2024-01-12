package org.firstinspires.ftc.teamcode.structures;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

// An extended version of the VisionPortal
public class VisionPortalEx extends VisionPortalImpl {
    public VisionPortalEx(CameraName camera, int cameraMonitorViewId, boolean autoPauseCameraMonitor, Size cameraResolution, StreamFormat webcamStreamFormat, VisionProcessor[] processors) {
        super(camera, cameraMonitorViewId, autoPauseCameraMonitor, cameraResolution, webcamStreamFormat, processors);
    }

    public OpenCvCamera getActiveCameraRaw(){
        return camera;
    }

    public static class Builder extends VisionPortal.Builder
    {
        // STATIC !
        private static final ArrayList<VisionProcessor> attachedProcessors = new ArrayList<>();

        private CameraName camera;
        private int liveViewContainerId = DEFAULT_VIEW_CONTAINER_ID; // 0 == none
        private boolean autoStopLiveView = true;
        private Size cameraResolution = new Size(640, 480);
        private StreamFormat streamFormat = null;
        private StreamFormat STREAM_FORMAT_DEFAULT = StreamFormat.YUY2;
        private final List<VisionProcessor> processors = new ArrayList<>();

        /**
         * Configure the portal to use a webcam
         * @param camera the WebcamName of the camera to use
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setCamera(CameraName camera)
        {
            this.camera = camera;
            return this;
        }

        /**
         * Configure the portal to use an internal camera
         * @param cameraDirection the internal camera to use
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setCamera(BuiltinCameraDirection cameraDirection)
        {
            this.camera = ClassFactory.getInstance().getCameraManager().nameFromCameraDirection(cameraDirection);
            return this;
        }

        /**
         * Configure the vision portal to stream from the camera in a certain image format
         * THIS APPLIES TO WEBCAMS ONLY!
         * @param streamFormat the desired streaming format
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setStreamFormat(StreamFormat streamFormat)
        {
            this.streamFormat = streamFormat;
            return this;
        }

        /**
         * Configure the vision portal to use (or not to use) a live camera preview
         * @param enableLiveView whether or not to use a live preview
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder enableLiveView(boolean enableLiveView)
        {
            int viewId;
            if (enableLiveView)
            {
                viewId = DEFAULT_VIEW_CONTAINER_ID;
            }
            else
            {
                viewId = 0;
            }
            return setLiveViewContainerId(viewId);
        }

        /**
         * Configure whether the portal should automatically pause the live camera
         * view if all attached processors are disabled; this can save computational resources
         * @param autoPause whether to enable this feature or not
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setAutoStopLiveView(boolean autoPause)
        {
            this.autoStopLiveView = autoPause;
            return this;
        }

        /**
         * A more advanced version of {@link #enableLiveView(boolean)}; allows you
         * to specify a specific view ID to use as a container, rather than just using the default one
         * @param liveViewContainerId view ID of container for live view
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setLiveViewContainerId(int liveViewContainerId)
        {
            this.liveViewContainerId = liveViewContainerId;
            return this;
        }

        /**
         * Specify the resolution in which to stream images from the camera. To find out what resolutions
         * your camera supports, simply call this with some random numbers (e.g. new Size(4634, 11115))
         * and the error message will provide a list of supported resolutions.
         * @param cameraResolution the resolution in which to stream images from the camera
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         */
        public Builder setCameraResolution(Size cameraResolution)
        {
            this.cameraResolution = cameraResolution;
            return this;
        }

        /**
         * Send a {@link VisionProcessor} into this portal to allow it to process camera frames.
         * @param processor the processor to attach
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         * @throws RuntimeException if the specified processor is already inside another portal
         */
        public Builder addProcessor(VisionProcessor processor)
        {
            synchronized (attachedProcessors)
            {
                if (attachedProcessors.contains(processor))
                {
                    throw new RuntimeException("This VisionProcessor has already been attached to a VisionPortal, either a different one or perhaps even this same portal.");
                }
                else
                {
                    attachedProcessors.add(processor);
                }
            }

            processors.add(processor);
            return this;
        }

        /**
         * Send multiple {@link VisionProcessor}s into this portal to allow them to process camera frames.
         * @param processors the processors to attach
         * @return the {@link VisionPortal.Builder} object, to allow for method chaining
         * @throws RuntimeException if the specified processor is already inside another portal
         */
        public Builder addProcessors(VisionProcessor... processors)
        {
            for (VisionProcessor p : processors)
            {
                addProcessor(p);
            }

            return this;
        }

        /**
         * Actually create the {@link VisionPortal} i.e. spool up the camera and live view
         * and begin sending image data to any attached {@link VisionProcessor}s
         * @return a configured, ready to use portal
         * @throws RuntimeException if you didn't specify what camera to use
         * @throws IllegalStateException if you tried to set the stream format when not using a webcam
         */
        public VisionPortalEx build()
        {
            if (camera == null)
            {
                throw new RuntimeException("You can't build a vision portal without setting a camera!");
            }

            if (streamFormat != null)
            {
                if (!camera.isWebcam() && !camera.isSwitchable())
                {
                    throw new IllegalStateException("setStreamFormat() may only be used with a webcam");
                }
            }
            else
            {
                // Only used with webcams, will be ignored for internal camera
                streamFormat = STREAM_FORMAT_DEFAULT;
            }

            VisionPortalEx portal = new VisionPortalEx(
                    camera, liveViewContainerId, autoStopLiveView, cameraResolution, streamFormat,
                    processors.toArray(new VisionProcessor[processors.size()]));

            // Clear this list to allow safe re-use of the builder object
            processors.clear();

            return portal;
        }
    }
}
