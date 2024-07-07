package org.firstinspires.ftc.teamcode.swerve.odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.core.RobotConfiguration;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 48/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -5.19; // X is the up and down direction
    public static double PARALLEL_Y = -0.77; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.39;
    public static double PERPENDICULAR_Y = -0.05;

    private Supplier<Double> headingSupplier;
    private Supplier<Double> headingVelocitySupplier;

    public static double X_MULTIPLIER = 0.9304872451; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9318336001; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    public TwoWheelTrackingLocalizer() {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        parallelEncoder = RobotConfiguration.ODOMETRY_PARALLEL_LEFT.getAsEncoder();
        perpendicularEncoder = RobotConfiguration.ODOMETRY_PERPENDICULAR.getAsEncoder();
    }

    public void setSupplier(Supplier<Double> headingSupplier, Supplier<Double> headingVelocitySupplier) {
        this.headingSupplier = headingSupplier;
        this.headingVelocitySupplier = headingVelocitySupplier;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return headingSupplier.get();
    }

    @Override
    public Double getHeadingVelocity() {
        return headingVelocitySupplier.get();
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
                encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
