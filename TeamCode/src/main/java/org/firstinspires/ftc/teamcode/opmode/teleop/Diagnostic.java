package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

@TeleOp(name = "Diagnostic", group = "C")
public class Diagnostic extends Manual {


    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;

    @Override
    public void init() {
        super.init();

        poseSupplier = swerveDrive::getPose;
        speedsSupplier = swerveDrive::getRobotVelocity;
    }

    @Override
    public void loop() {
        super.loop();

        telemetry.addData("Drive Pose", poseSupplier.get());

        for(RobotConfiguration configuration : RobotConfiguration.values()) {
            HardwareDevice device = configuration.getAsHardwareDevice();
            if(device instanceof Encoder)
                telemetry.addData(configuration.name() + " Encoder", ((Encoder) device).getCurrentPosition());
            else if(device instanceof AbsoluteEncoder)
                telemetry.addData(configuration.name() + " Abs", ((AbsoluteEncoder) device).getCurrentPosition());

        }
        
    }
}