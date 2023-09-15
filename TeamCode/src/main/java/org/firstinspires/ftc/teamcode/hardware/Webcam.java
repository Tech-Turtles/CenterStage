package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class Webcam extends HardwareDevice {

    private WebcamName device;
    private VisionPortal visionPortal;
    private VisionProcessor visionProcessor;

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

        setStatus(HardwareStatus.SUCCESS);
    }

    public void stop() {

    }
}