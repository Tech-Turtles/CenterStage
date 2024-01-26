package org.firstinspires.ftc.teamcode.swerve.odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.Encoder;

import java.util.Arrays;
import java.util.List;

public class ThreeTrackingWheelLocalizer extends com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.74803; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.72; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0.0; // in; offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double X_MULTIPLIER = 0.9304872451; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9318336001; // Multiplier in the Y direction

    public ThreeTrackingWheelLocalizer() {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = RobotConfiguration.ODOMETRY_PARALLEL_LEFT.getAsEncoder();
        frontEncoder = RobotConfiguration.ODOMETRY_PERPENDICULAR.getAsEncoder();
        rightEncoder = RobotConfiguration.ODOMETRY_PARALLEL_RIGHT.getAsEncoder();
    }

    public static double encoderTicksToInches(double ticks) {
        return (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );

    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        //  If the encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}