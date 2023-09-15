package org.firstinspires.ftc.teamcode.utility.math.geometry;


import org.firstinspires.ftc.teamcode.utility.math.Matrix;
import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Vector;
import org.firstinspires.ftc.teamcode.utility.math.interpolation.Interpolatable;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N1;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N3;

import java.util.Objects;

/** Represents a 3D pose containing translational and rotational elements. */
public class Pose3d implements Interpolatable<Pose3d> {
    private final Translation3d m_translation;
    private final Rotation3d m_rotation;

    /** Constructs a pose at the origin facing toward the positive X axis. */
    public Pose3d() {
        m_translation = new Translation3d();
        m_rotation = new Rotation3d();
    }

    /**
     * Constructs a pose with the specified translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose3d(Translation3d translation, Rotation3d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs a pose with x, y, and z translations instead of a separate Translation3d.
     *
     * @param x The x component of the translational component of the pose.
     * @param y The y component of the translational component of the pose.
     * @param z The z component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        m_translation = new Translation3d(x, y, z);
        m_rotation = rotation;
    }

    /**
     * Constructs a 3D pose from a 2D pose in the X-Y plane.
     *
     * @param pose The 2D pose.
     */
    public Pose3d(Pose2d pose) {
        m_translation = new Translation3d(pose.getX(), pose.getY(), 0.0);
        m_rotation = new Rotation3d(0.0, 0.0, pose.getRotation().getRadians());
    }

    /**
     * Transforms the pose by the given transformation and returns the new transformed pose.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose3d plus(Transform3d other) {
        return transformBy(other);
    }

    /**
     * Returns the Transform3d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    public Transform3d minus(Pose3d other) {
        final Pose3d pose = this.relativeTo(other);
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the pose.
     */
    public Translation3d getTranslation() {
        return m_translation;
    }

    /**
     * Returns the X component of the pose's translation.
     *
     * @return The x component of the pose's translation.
     */
    public double getX() {
        return m_translation.getX();
    }

    /**
     * Returns the Y component of the pose's translation.
     *
     * @return The y component of the pose's translation.
     */
    public double getY() {
        return m_translation.getY();
    }

    /**
     * Returns the Z component of the pose's translation.
     *
     * @return The z component of the pose's translation.
     */
    public double getZ() {
        return m_translation.getZ();
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return The rotational component of the pose.
     */
    public Rotation3d getRotation() {
        return m_rotation;
    }

    /**
     * Multiplies the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose3d.
     */
    public Pose3d times(double scalar) {
        return new Pose3d(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Divides the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose3d.
     */
    public Pose3d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Transforms the pose by the given transformation and returns the new pose.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose3d transformBy(Transform3d other) {
        return new Pose3d(
                m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
                other.getRotation().plus(m_rotation));
    }

    /**
     * Returns the current pose relative to the given pose.
     *
     * <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
     * get the error between the reference and the current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that the current pose will
     *     be converted into.
     * @return The current pose relative to the new origin pose.
     */
    public Pose3d relativeTo(Pose3d other) {
        Transform3d transform = new Transform3d(other, this);
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    // Implement the RotationVectorToMatrix function for 3D rotation vectors
    public static Matrix<N3, N3> rotationVectorToMatrix(Matrix<N3, N1> rotation) {
        double a = rotation.get(0, 0);
        double b = rotation.get(1, 0);
        double c = rotation.get(2, 0);

        // Construct the 3x3 rotation matrix from the given rotation vector
        Matrix<N3, N3> x = new Matrix<>(Nat.N3(), Nat.N3());
        x.set(1, 1, 0.0);
        x.set(1, 2, -c);
        x.set(1, 3, b);

        x.set(2, 1, c);
        x.set(2, 2, 0.0);
        x.set(2, 3, -a);

        x.set(3, 1, -b);
        x.set(3, 2, a);
        x.set(3, 3, 0.0);

        return x;
    }

    // Implement the Exp method for Pose3d using the given Twist3d
    public Pose3d exp(Twist3d twist) {
        Matrix<N3, N1> u = new Matrix<>(Nat.N3(), Nat.N1());
        u.set(1, 1, twist.dx);
        u.set(2, 1, twist.dy);
        u.set(3, 1, twist.dz);

        Matrix<N3, N1> rvec = new Matrix<>(Nat.N3(), Nat.N1());
        rvec.set(1, 1, twist.rx);
        rvec.set(2, 1, twist.ry);
        rvec.set(3, 1, twist.rz);

        Matrix<N3, N3> omega = rotationVectorToMatrix(rvec);
        Matrix<N3, N3> omegaSq = omega.times(omega);
        double theta = rvec.normF();
        double thetaSq = theta * theta;

        double A;
        double B;
        double C;
        if (Math.abs(theta) < 1E-7) {
            // Taylor Expansions around θ = 0
            // A = 1/1! - θ²/3! + θ⁴/5!
            // B = 1/2! - θ²/4! + θ⁴/6!
            // C = 1/3! - θ²/5! + θ⁴/7!
            A = 1 - thetaSq / 6 + thetaSq * thetaSq / 120;
            B = 1 / 2.0 - thetaSq / 24 + thetaSq * thetaSq / 720;
            C = 1 / 6.0 - thetaSq / 120 + thetaSq * thetaSq / 5040;
        } else {
            // A = sin(θ)/θ
            // B = (1 - cos(θ)) / θ²
            // C = (1 - A) / θ²
            A = Math.sin(theta) / theta;
            B = (1 - Math.cos(theta)) / thetaSq;
            C = (1 - A) / thetaSq;
        }

        Matrix<N3, N3> R = Matrix.eye(Nat.N3()).plus(omega.times(A)).plus(omegaSq.times(B));
        Matrix<N3, N3> V = Matrix.eye(Nat.N3()).plus(omega.times(B)).plus(omegaSq.times(C));

        Matrix<N3, N1> translationComponent = V.times(u);
        Transform3d transform = new Transform3d(
                new Translation3d(translationComponent.get(0, 0), translationComponent.get(1, 0), translationComponent.get(2, 0)),
                new Rotation3d(R));

        // Return the new Pose3d object after the exponential transformation
        return this.plus(transform);
    }

    // Implement the Log method for Pose3d using the given end Pose3d
    public Twist3d log(Pose3d end) {
        Pose3d transform = end.relativeTo(this);

        Matrix<N3, N1> u = new Matrix<>(Nat.N3(), Nat.N1());
        u.set(1, 1, transform.getX());
        u.set(2, 1, transform.getY());
        u.set(3, 1, transform.getZ());

        Vector<N3> rvec = transform.getRotation().getQuaternion().toRotationVector();

        Matrix<N3, N3> omega = rotationVectorToMatrix(rvec);
        Matrix<N3, N3> omegaSq = omega.times(omega);
        double theta = rvec.normF();
        double thetaSq = theta * theta;

        double C;
        if (Math.abs(theta) < 1E-7) {
            // Taylor Expansions around θ = 0
            // A = 1/1! - θ²/3! + θ⁴/5!
            // B = 1/2! - θ²/4! + θ⁴/6!
            // C = 1/6 * (1/2 + θ²/5! + θ⁴/7!)
            C = 1 / 12.0 + thetaSq / 720 + thetaSq * thetaSq / 30240;
        } else {
            // A = sin(θ)/θ
            // B = (1 - cos(θ)) / θ²
            // C = (1 - A/(2*B)) / θ²
            double A = Math.sin(theta) / theta;
            double B = (1 - Math.cos(theta)) / thetaSq;
            C = (1 - A / (2 * B)) / thetaSq;
        }

        Matrix<N3, N3> V_inv = Matrix.eye(Nat.N3()).minus(omega.times(0.5)).plus(omegaSq.times(C));

        Matrix<N3, N1> translation_component = V_inv.times(u);

        return new Twist3d(translation_component.get(0, 0), translation_component.get(1, 0), translation_component.get(2, 0),
                rvec.get(0, 0), rvec.get(1, 0), rvec.get(2, 0));
    }

    /**
     * Returns a Pose2d representing this Pose3d projected into the X-Y plane.
     *
     * @return A Pose2d representing this Pose3d projected into the X-Y plane.
     */
    public Pose2d toPose2d() {
        return new Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d());
    }

    @Override
    public String toString() {
        return String.format("Pose3d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Pose3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Pose3d) {
            return ((Pose3d) obj).m_translation.equals(m_translation)
                    && ((Pose3d) obj).m_rotation.equals(m_rotation);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }

    @Override
    public Pose3d interpolate(Pose3d endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            Twist3d twist = this.log(endValue);
            Twist3d scaledTwist =
                    new Twist3d(
                            twist.dx * t, twist.dy * t, twist.dz * t, twist.rx * t, twist.ry * t, twist.rz * t);
            return this.exp(scaledTwist);
        }
    }
}
