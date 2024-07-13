package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.swerve.odometry.AprilTagPosition;
import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.math.util.Units;
import org.firstinspires.ftc.teamcode.utility.misc.TimingMonitor;
import org.firstinspires.ftc.teamcode.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.GoalEndState;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathConstraints;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.utility.pathplanner.util.PIDConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Consumer;
import java.util.function.Supplier;

@TeleOp(name="Vision", group="B")
public class Vision extends Manual {
    private final Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();
    private ExecutorService executorService;
    private Future<?> detectionFuture;
//    private TimingMonitor timingMonitor;

    private volatile Pose2d latestRobotPose;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    boolean hasRun = false;
    boolean finishedDriving = true;
    ElapsedTimer elapsedTimer = new ElapsedTimer();

    boolean useAprilTags = true;
    Pose2d aprilTagPose = new Pose2d();

    @Override
    public void init() {
        super.init();
        webcam.enableAprilTagProcessor();
        executorService = Executors.newSingleThreadExecutor();
//        timingMonitor = new TimingMonitor(this);

        controller = new PPHolonomicDriveController(
                new PIDConstants(6.0, 0.0, 0.0),
                new PIDConstants(8.0, 0.0, 0.0),
                0.017,
                1.6,
                0.2507
        );

        poseSupplier = swerveDrive::getPose;
        speedsSupplier = swerveDrive::getRobotVelocity;
        output = swerveDrive::drive;

        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        controller.reset(poseSupplier.get(), currentSpeeds);
        elapsedTimer.reset();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
//        timingMonitor.enable();
    }

    @Override
    public void loop() {
        super.loop();

        if(primary.rightBumperOnce())
            useAprilTags = !useAprilTags;

        telemetry.addData("Use Tags", useAprilTags);

//        timingMonitor.loopStart();
//        timingMonitor.checkpoint("LOOP");
        if(useAprilTags) {
            // Check if previous detection processing is done
            if (detectionFuture == null || detectionFuture.isDone()) {
                // Start new detection processing
                detectionFuture = executorService.submit(this::processDetections);
            }
//        timingMonitor.checkpoint("DETECTION");

            // Apply the latest robot pose if available
            if (latestRobotPose != null) {
                swerveDrive.resetOdometry(latestRobotPose);
                aprilTagPose = latestRobotPose;
                latestRobotPose = null;
            }
//            getRobotPoseFromTag(webcam.getAprilTagProcessor().getDetections());
        }

//        timingMonitor.checkpoint("RESET");

        telemetry.addData("Drive Pose", swerveDrive.getPose());
        packet.put("Drive Pose", swerveDrive.getPose());
        telemetry.addData("April Tag Pose", aprilTagPose);
        packet.put("April Tag Pose", aprilTagPose);

        if(primary.leftBumperOnce()) {
            // Create a list of bezier points from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    swerveDrive.getPose(),
                    new Pose2d(1.8, 4.3, Rotation2d.fromDegrees(0))
            );

        // Create the path using the bezier points created above
            path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(1.0, 1.0, 1 * Math.PI, 2 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                    new GoalEndState(0.0, swerveDrive.getYaw()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            trajectory = new PathPlannerTrajectory(path, swerveDrive.getRobotVelocity());
            finishedDriving = false;
            hasRun = false;
        }

        if(finishedDriving) {
            telemetry.addLine("Finished Driving");
            return;
        }

        if (!hasRun) {
            elapsedTimer.reset();
            hasRun = true;
        }

        double currentTime = elapsedTimer.seconds();
        PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
        output.accept(targetSpeeds);

        if (currentTime > trajectory.getTotalTimeSeconds()) {
            finishedDriving = true;
            output.accept(new ChassisSpeeds(0,0,0));
        }

//        timingMonitor.displayMaxTimes();
    }

    @Override
    public void stop() {
        super.stop();
        if (executorService != null) {
            executorService.shutdown();
        }
    }

    private void processDetections() {
        try {
            latestRobotPose = getRobotPoseFromTag(webcam.getAprilTagProcessor().getDetections());
        } catch (Exception ignore) {
        }
    }

    private Transform2d robotToCamera = new Transform2d(new Translation2d(Units.inchesToMeters(-6.715), 0.0),
            new Rotation2d(Math.toRadians(180.0)));

    private static final Pose2d CAMERA_POSE= new Pose2d(0.0, 8.715, new Rotation2d());

    private Pose2d getRobotPoseFromTag(ArrayList<AprilTagDetection> currentDetections) {
        List<Pose2d> backdropPositions = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addData("X " + detection.id, (detection.ftcPose.x));
//                telemetry.addData("Y " + detection.id, (detection.ftcPose.y));
//                telemetry.addData("Heading " + detection.id, (-detection.ftcPose.yaw));

                Rotation2d heading = Rotation2d.fromDegrees(detection.ftcPose.yaw);
                double x = detection.ftcPose.x * heading.getCos() - detection.ftcPose.y * heading.getSin();
                double y = detection.ftcPose.x * heading.getSin() + detection.ftcPose.y * heading.getCos();
                Pose2d pose = new Pose2d(x, y, heading);
                switch (detection.id) {
                    case 1:
                    case 4:
                        pose = pose.add(new Pose2d(6, 0, new Rotation2d()));
                        break;
                    case 3:
                    case 6:
                        pose = pose.add(new Pose2d(-6, 0, new Rotation2d()));
                        break;
                    default:
                        break;
                }
                backdropPositions.add(pose);

//                telemetry.addData("Field pos " + detection.id, pose);
            }
        }

        Pose2d backdropPosition = backdropPositions.stream().reduce(Pose2d::add).orElse(new Pose2d());
        backdropPosition = backdropPosition.times(1.0 / backdropPositions.size());


        Pose2d globalTagPosition = new Pose2d();
        Pose2d positionWithOffset = new Pose2d();
        if (swerveDrive.getPose().getX() > 1.8) {
            Translation2d offset = new Translation2d(
                    CAMERA_POSE.getX() * Math.cos(backdropPosition.getRotation().getRadians()) - CAMERA_POSE.getY() * Math.sin(backdropPosition.getRotation().getRadians()),
                    CAMERA_POSE.getX() * Math.sin(backdropPosition.getRotation().getRadians()) + CAMERA_POSE.getY() * Math.cos(backdropPosition.getRotation().getRadians())
            );

            positionWithOffset = backdropPosition.add(new Pose2d(offset.getX(), offset.getY(), new Rotation2d()));
            //telemetry.addData("Robot Center to Tag", positionWithOffset);
            globalTagPosition = positionWithOffset.subtract(new Pose2d(100.0f + 6.0f, 179.672796f, new Rotation2d()));
        } else {
            Translation2d offset = new Translation2d(
                    CAMERA_POSE.getX() * Math.cos(backdropPosition.getRotation().getRadians()) - CAMERA_POSE.getY() * Math.sin(backdropPosition.getRotation().getRadians()),
                    CAMERA_POSE.getX() * Math.sin(backdropPosition.getRotation().getRadians()) + CAMERA_POSE.getY() * Math.cos(backdropPosition.getRotation().getRadians())
            );

            positionWithOffset = backdropPosition.add(new Pose2d(offset.getX(), offset.getY(), new Rotation2d()));
//            telemetry.addData("Robot Center to Tag", positionWithOffset);
            globalTagPosition = positionWithOffset.subtract(new Pose2d(29.46f + 6.0f, 179.672796f, new Rotation2d()));
        }
        if (Double.isNaN(globalTagPosition.getX()) || Double.isNaN(globalTagPosition.getY()) || Double.isNaN(globalTagPosition.getRotation().getRadians())) return null;
//        telemetry.addData("Global tag position", globalTagPosition);
        Rotation2d test = new Rotation2d(globalTagPosition.getRotation().getRadians() + Math.toRadians(270.0));
        double compPos = test.getDegrees();
        compPos += compPos < 0 ? 360.0 : (compPos > 360.0 ? -360.0 : 0.0);
        return new Pose2d(globalTagPosition.getX() * -0.0254, globalTagPosition.getY() * -0.0254, Rotation2d.fromDegrees(compPos));
    }
}