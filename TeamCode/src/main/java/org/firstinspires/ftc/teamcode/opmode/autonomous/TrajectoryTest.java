package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.swerve.SwerveDriveController;
import org.firstinspires.ftc.teamcode.utility.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.utility.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.TrajectoryConfig;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.TrapezoidProfile;

import java.util.stream.Collectors;
import java.util.stream.Stream;

@Config
@Autonomous(name = "Trajectory Test", group = "E")
public class TrajectoryTest extends RobotHardware {

    SwerveDriveController controller;
    public static double x1 = 0.5, y1 = 0.5, x2 = 1.0, y2 = 0.5, endX = 1.1, endY = 0.5;
    private Trajectory test;

    @Override
    public void init() {
        super.init();
        TrajectoryConfig config = new TrajectoryConfig(1.5, 0.75)
                .setKinematics(swerveDrive.kinematics);
        test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                Stream.of(
                        new Translation2d(x1, y1),
                        new Translation2d(x2, y2)
                ).collect(Collectors.toList()),
                new Pose2d(endX, endY, Rotation2d.fromDegrees(0.0)),
                config
        );

        TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        swerveControllerConfiguration.maxAngularVelocity,
                        Math.PI / 4);

        controller = new SwerveDriveController(
                test,
                swerveDrive::getPose,
                swerveDrive.kinematics,
                new PIDController(1.5, 0.0, 0.0, 0.001),
                new PIDController(1.5, 0.0, 0.0, 0.001),
                new ProfiledPIDController(3.0, 0.0, 0.0, kThetaControllerConstraints, 0.001),
                swerveDrive::setModuleStates
        );

        controller.initialize();
    }

    @Override
    public void init_loop() {
        super.init_loop();

    }

    @Override
    public void start() {
        super.start();

    }

    @Override
    public void loop() {
        super.loop();
        controller.update();

        if(controller.isFinished()) {
            swerveDrive.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
            stop();
        }
    }
}