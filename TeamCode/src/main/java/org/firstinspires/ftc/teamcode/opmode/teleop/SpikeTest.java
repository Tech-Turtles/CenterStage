package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.vision.SpikeProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Spike Test")
public class SpikeTest extends Manual {
    private WebcamName spikeCamera;
    private SpikeProcessor spikeProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        super.init();
        spikeCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        spikeProcessor = new SpikeProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(spikeCamera)
                .addProcessor(spikeProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
//                .setLiveViewContainerId(liveViewContainerId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(spikeProcessor, true);
    }

    @Override
    public void loop() {
        super.loop();

        try {
            telemetry.addData("Spike Position", spikeProcessor.location);
            packet.put("Spike Position", spikeProcessor.location);
        } catch (Exception ignore) {}
    }
}
