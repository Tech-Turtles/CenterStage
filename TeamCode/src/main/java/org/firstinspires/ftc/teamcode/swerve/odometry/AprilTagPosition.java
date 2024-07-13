package org.firstinspires.ftc.teamcode.swerve.odometry;

import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Quaternion;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utility.math.util.Units;

public enum AprilTagPosition {
    NONE(new Pose3d()),
    BLUE_LEFT(new Pose2d(Units.inchesToMeters(29.16f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    BLUE_CENTER(new Pose2d(Units.inchesToMeters(29.16f + 6.0f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    BLUE_RIGHT(new Pose2d(Units.inchesToMeters(29.16f + 12.0f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    RED_LEFT(new Pose2d(Units.inchesToMeters(100.0f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    RED_CENTER(new Pose2d(Units.inchesToMeters(100.0f + 6.0f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    RED_RIGHT(new Pose2d(Units.inchesToMeters(100.0f + 12.0f), Units.inchesToMeters(179.672796f), Rotation2d.fromDegrees(180.0))),
    RED_WALL_LARGE(new Pose3d(
            Units.inchesToMeters(60.25f), Units.inchesToMeters(41.41f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    RED_WALL_SMALL(new Pose3d(
            Units.inchesToMeters(60.25f), Units.inchesToMeters(41.41f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    BLUE_WALL_SMALL(new Pose3d(
            Units.inchesToMeters(60.25f), Units.inchesToMeters(41.41f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    BLUE_WALL_LARGE(new Pose3d(
            Units.inchesToMeters(60.25f), Units.inchesToMeters(41.41f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    SHARED_LEFT(new Pose3d(
            Units.inchesToMeters(64.6f), Units.inchesToMeters(179.672796f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    SHARED_CENTER(new Pose3d(
            Units.inchesToMeters(70.6f), Units.inchesToMeters(179.672796f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f)))),
    SHARED_RIGHT(new Pose3d(
            Units.inchesToMeters(76.6f), Units.inchesToMeters(179.672796f), Units.inchesToMeters(4f),
            new Rotation3d(new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f))));

    private Pose3d pose;
    private Pose2d pose2d;

    AprilTagPosition(Pose3d pose) {
        this.pose = pose;
    }

    AprilTagPosition(Pose2d pose) {
        this.pose2d = pose;
    }

    public static AprilTagPosition getTagPose(int detection) {
        for (AprilTagPosition tag : AprilTagPosition.values()) {
            if(tag.ordinal() == detection)
                return tag;
        }
        return NONE;
    }

    public Pose3d getPose() {
        return pose;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }
}