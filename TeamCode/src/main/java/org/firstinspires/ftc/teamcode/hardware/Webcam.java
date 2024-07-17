package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//ToDo (Low Priority) Allow for multiple vision processors to be passed
public class Webcam extends HardwareDevice {

    private WebcamName device, spikeCamera;
    private VisionPortal visionPortal;
    private VisionProcessor visionProcessor;
    private Size resolution = new Size(640, 480);
    private int liveViewContainerId = 0; // 0 == none
    private VisionPortal.StreamFormat streamFormat = VisionPortal.StreamFormat.MJPEG;
    private AprilTagProcessor aprilTag;

    public Webcam(String configName) {
        super(configName, WebcamName.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.device = (WebcamName) device;

        visionProcessor = new SpikeDetectionProcessor();

//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(this.device, spikeCamera);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(this.device)
                .addProcessor(visionProcessor)
//                .setCameraResolution(resolution)
                .enableLiveView(true)
                .setStreamFormat(streamFormat);

//        builder.setCameraResolution(new Size(640, 480));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        visionPortal.setProcessorEnabled(aprilTag, false);

        setStatus(HardwareStatus.SUCCESS);
    }

    public void setSpikeCamera(WebcamName spikeCamera) {
        this.spikeCamera = spikeCamera;
    }

    public Webcam configureVisionProcessor(VisionProcessor visionProcessor) {
        this.visionProcessor = visionProcessor;
        return this;
    }

    public Webcam configureCameraResolution(int width, int height) {
        return configureCameraResolution(new Size(width, height));
    }

    public Webcam configureCameraResolution(Size resolution) {
        this.resolution = resolution;
        return this;
    }

    public Webcam configureStreamFormat(VisionPortal.StreamFormat streamFormat) {
        this.streamFormat = streamFormat;
        return this;
    }

    public void disableProcessor() {
        if(visionPortal != null && visionPortal.getProcessorEnabled(visionProcessor))
            visionPortal.setProcessorEnabled(visionProcessor, false);
    }

    public void enableProcessor() {
        if(visionPortal != null && !visionPortal.getProcessorEnabled(visionProcessor))
            visionPortal.setProcessorEnabled(visionProcessor, true);
    }

    public SpikeDetectionProcessor getProcessor() {
        return (SpikeDetectionProcessor) visionProcessor;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public void enableAprilTagProcessor() {
        disableProcessor();
        if(visionPortal != null && !visionPortal.getProcessorEnabled(aprilTag))
            visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void disableAprilTagProcessor() {
        disableProcessor();
        if(visionPortal != null && visionPortal.getProcessorEnabled(aprilTag))
            visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void enableSpikeCamera() {
        if(visionPortal != null) {
            visionPortal.setActiveCamera(spikeCamera);
            disableAprilTagProcessor();
            enableProcessor();
        }
    }

    public void enableAprilTagCamera() {
        if(visionPortal != null) {
            visionPortal.setActiveCamera(device);
            disableProcessor();
            enableAprilTagProcessor();
        }
    }

    public void stop() {
        if(visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}