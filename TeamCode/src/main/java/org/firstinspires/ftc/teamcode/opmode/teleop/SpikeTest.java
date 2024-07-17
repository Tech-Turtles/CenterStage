package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Spike Test")
public class SpikeTest extends OpMode {
    private WebcamName spikeCamera, tagCamera;
    private SpikeDetectionProcessor spikeProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        tagCamera = hardwareMap.get(WebcamName.class, "Webcam");
        spikeCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        spikeProcessor = new SpikeDetectionProcessor();

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(tagCamera, spikeCamera);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(spikeProcessor)
                .enableLiveView(true)
//                .setLiveViewContainerId(liveViewContainerId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(spikeProcessor, true);
    }

    @Override
    public void start() {
        super.start();
        visionPortal.setActiveCamera(spikeCamera);
    }

    @Override
    public void loop() {
        try {
            telemetry.addData("Spike Position", spikeProcessor.location);
        } catch (Exception ignore) {}
    }
}
