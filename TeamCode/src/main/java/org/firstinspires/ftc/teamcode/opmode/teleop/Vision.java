package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Webcam;

@TeleOp(name="Vision", group="B")
public class Vision extends Manual {
    Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        try {
            telemetry.addData("Red Rect", webcam.getProcessor().getRedRect());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        try {
            telemetry.addData("Blue Rect", webcam.getProcessor().getBlueRect());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
