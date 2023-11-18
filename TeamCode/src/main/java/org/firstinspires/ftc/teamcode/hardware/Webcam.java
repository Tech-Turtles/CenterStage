package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

//ToDo (Low Priority) Allow for multiple vision processors to be passed
public class Webcam extends HardwareDevice {

    private WebcamName device;
    private VisionPortal visionPortal;
    private VisionProcessor visionProcessor;
    private Size resolution = new Size(640, 480);
    private int liveViewContainerId = 0; // 0 == none
    private VisionPortal.StreamFormat streamFormat = VisionPortal.StreamFormat.MJPEG;

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

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(this.device)
                .addProcessor(visionProcessor)
                .setCameraResolution(resolution)
//                .setLiveViewContainerId(liveViewContainerId)
                .setStreamFormat(streamFormat);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(visionProcessor, true);


        setStatus(HardwareStatus.SUCCESS);
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

    public void stop() {
        if(visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}