package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;

@TeleOp(name = "Diagnostic")
public class Diagnostic extends Manual {


    @Override
    public void loop() {
        super.loop();

        telemetry.addData("Drive Pose", swerveDrive.getPose());

        for(RobotConfiguration configuration : RobotConfiguration.values()) {
            HardwareDevice device = configuration.getAsHardwareDevice();
            if(device instanceof Encoder)
                telemetry.addData(configuration.name() + " Encoder", ((Encoder) device).getCurrentPosition());
//            else if(device instanceof AbsoluteEncoder)
//                telemetry.addData(configuration.name() + " Abs", ((AbsoluteEncoder) device).getCurrentPosition());
        }
    }
}