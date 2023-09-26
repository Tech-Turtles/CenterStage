package org.firstinspires.ftc.teamcode.utility.math.trajectory.constraint;

import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;

/**
 * An interface for defining user-defined velocity and acceleration constraints while generating
 * trajectories.
 */
public interface TrajectoryConstraint {
    /**
     * Returns the max velocity given the current pose and curvature.
     *
     * @param poseMeters The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
     * @param velocityMetersPerSecond The velocity at the current point in the trajectory before
     *     constraints are applied.
     * @return The absolute maximum velocity.
     */
    double getMaxVelocityMetersPerSecond(
            Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond);

    /**
     * Returns the minimum and maximum allowable acceleration for the trajectory given pose,
     * curvature, and speed.
     *
     * @param poseMeters The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
     * @param velocityMetersPerSecond The speed at the current point in the trajectory.
     * @return The min and max acceleration bounds.
     */
    MinMax getMinMaxAccelerationMetersPerSecondSq(
            Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond);

    /** Represents a minimum and maximum acceleration. */
    class MinMax {
        public double minAccelerationMetersPerSecondSq = -Double.MAX_VALUE;
        public double maxAccelerationMetersPerSecondSq = +Double.MAX_VALUE;

        /**
         * Constructs a MinMax.
         *
         * @param minAccelerationMetersPerSecondSq The minimum acceleration.
         * @param maxAccelerationMetersPerSecondSq The maximum acceleration.
         */
        public MinMax(
                double minAccelerationMetersPerSecondSq, double maxAccelerationMetersPerSecondSq) {
            this.minAccelerationMetersPerSecondSq = minAccelerationMetersPerSecondSq;
            this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
        }

        /** Constructs a MinMax with default values. */
        public MinMax() {}
    }
}
