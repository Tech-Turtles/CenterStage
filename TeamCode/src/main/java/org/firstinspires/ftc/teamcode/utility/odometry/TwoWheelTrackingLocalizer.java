package org.firstinspires.ftc.teamcode.utility.odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.IMU;

import java.util.Arrays;
import java.util.List;

@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.74803; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 1.75; // X is the up and down direction
    public static double PARALLEL_Y = 7.25; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6.75;
    public static double PERPENDICULAR_Y = 0.25;

    public static double X_MULTIPLIER = 1.0; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private IMU imu;

    public TwoWheelTrackingLocalizer() {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.imu = RobotConfiguration.IMU.getAsIMU();

        parallelEncoder = RobotConfiguration.ODOMETRY_PARALLEL_LEFT.getAsEncoder();
        perpendicularEncoder = RobotConfiguration.ODOMETRY_PERPENDICULAR.getAsEncoder();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imu.getRotation3d().getX();
    }

    @Override
    public Double getHeadingVelocity() {
        return imu.getXAngularVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }
}
