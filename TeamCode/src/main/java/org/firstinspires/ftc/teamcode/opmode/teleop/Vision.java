package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.swerve.odometry.AprilTagPosition;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation3d;
import org.firstinspires.ftc.teamcode.utility.math.util.Units;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import kotlin.Unit;

@TeleOp(name="Vision", group="B")
public class Vision extends Manual {
    Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();

    @Override
    public void init() {
        super.init();
        webcam.enableAprilTagProcessor();
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
//        try {
//            telemetry.addData("Red Rect", webcam.getProcessor().getRedRect());
//        } catch (Exception e) {
//            throw new RuntimeException(e);
//        }
//        try {
//            telemetry.addData("Blue Rect", webcam.getProcessor().getBlueRect());
//        } catch (Exception e) {
//            throw new RuntimeException(e);
//        }

        try {
            ArrayList<AprilTagDetection> detections = webcam.getAprilTagProcessor().getDetections();
            detections.sort(Comparator.comparingDouble(v -> v.ftcPose.range));
            if(!detections.isEmpty()) {
                AprilTagDetection closestDetection = detections.get(0);

                Pose2d robotPose = getRobotPoseFromTag(closestDetection, swerveDrive.getYaw().getDegrees());
                swerveDrive.resetOdometry(new Pose2d(robotPose.getY(), robotPose.getX(), Rotation2d.fromDegrees(0.0)));
            }
        } catch (Exception ignore) {}

        telemetry.addData("Drive Pose", swerveDrive.getPose());

    }

//    private Transform3d camConstant =
//            new Transform3d(new Translation3d(Units.inchesToMeters(-6.715), 0.0, Units.inchesToMeters(4.5)),
//                    new Rotation3d(0.0, 0.0, Math.toRadians(180)));
    private Transform2d robotToCamera = new Transform2d(new Translation2d(Units.inchesToMeters(-6.715), 0.0),
        new Rotation2d(Math.toRadians(180.0)));

    private Pose2d getRobotPoseFromTag(AprilTagDetection detection, double heading) {
        AprilTagPosition tag = AprilTagPosition.getTagPose(detection.id);

        Transform2d camToTarget = new Transform2d(
                new Translation2d(
                    Units.inchesToMeters(detection.ftcPose.x), Units.inchesToMeters(detection.ftcPose.y)),
                new Rotation2d(Math.atan(Units.inchesToMeters(detection.ftcPose.y) / Units.inchesToMeters(detection.ftcPose.x))));

        Pose2d robot = tag.getPose2d()
                .plus(new Transform2d(
                        new Translation2d(
                            Units.inchesToMeters(detection.ftcPose.x),
                            Units.inchesToMeters(detection.ftcPose.y)),
                        Rotation2d.fromRadians(detection.ftcPose.yaw)))
                .plus(new Transform2d(
                        new Translation2d(
                                Units.inchesToMeters(-6.715),
                                Units.inchesToMeters(0.0)),
                        Rotation2d.fromDegrees(180.0)
                ));

        Pose2d tagPose = tag.getPose2d();

        double tagToRobotX = Units.inchesToMeters(detection.ftcPose.x);
        double tagToRobotY = Units.inchesToMeters(detection.ftcPose.y);

        telemetry.addData("Tag relative pos " + detection.id, new Translation2d(tagToRobotX, tagToRobotY));

        double x = tagToRobotX - robotToCamera.getX();
        double y = tagToRobotY;

        Pose2d cameraPos = new Pose2d(new Translation2d(
                tagPose.getX() - tagToRobotX,
                tagPose.getY() + tagToRobotY
        ), new Rotation2d());

        telemetry.addData("Cam pos " + detection.id, cameraPos);

        double xwhat = cameraPos.getX() - Math.sin(heading) * robotToCamera.getX();
        double ywhat = cameraPos.getY() - Math.cos(heading) * robotToCamera.getX();

        return new Pose2d(new Translation2d(
                xwhat,
                ywhat
        ), new Rotation2d(0.0));
//        Transform3d bestCameraToTarget = new Transform3d(
//                new Translation3d(
//                        Units.inchesToMeters(detection.ftcPose.x),
//                        Units.inchesToMeters(detection.ftcPose.y),
//                        Units.inchesToMeters(detection.ftcPose.z)),
//                new Rotation3d(0.0, 0.0, Math.atan(detection.ftcPose.y / detection.ftcPose.x)));

//        return tag.getPose2d()
//                .transformBy(camToTarget.inverse())
//                .transformBy(robotToCamera.inverse());
//        return robot;
    }

    private String getPose(AprilTagPoseFtc pose) {
        return String.format("X: %.4f, Y: %.4f, Z: %.4f, Yaw: %.4f, Roll: %.4f, Pitch: %.4f", pose.x, pose.y, pose.z, pose.yaw, pose.roll, pose.pitch);
    }
}
