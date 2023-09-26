package org.firstinspires.ftc.teamcode.utility.math;

public final class WPIMathJNI {

    /**
     * Solves the discrete algebraic Riccati equation.
     *
     * @param A Array containing elements of A in row-major order.
     * @param B Array containing elements of B in row-major order.
     * @param Q Array containing elements of Q in row-major order.
     * @param R Array containing elements of R in row-major order.
     * @param states Number of states in A matrix.
     * @param inputs Number of inputs in B matrix.
     * @param S Array storage for DARE solution.
     */
    public static native void dare(
            double[] A, double[] B, double[] Q, double[] R, int states, int inputs, double[] S);

    /**
     * Computes the matrix exp.
     *
     * @param src Array of elements of the matrix to be exponentiated.
     * @param rows How many rows there are.
     * @param dst Array where the result will be stored.
     */
    public static native void exp(double[] src, int rows, double[] dst);

    /**
     * Computes the matrix pow.
     *
     * @param src Array of elements of the matrix to be raised to a power.
     * @param rows How many rows there are.
     * @param exponent The exponent.
     * @param dst Array where the result will be stored.
     */
    public static native void pow(double[] src, int rows, double exponent, double[] dst);

    /**
     * Obtain a Pose3d from a (constant curvature) velocity.
     *
     * <p>The double array returned is of the form [dx, dy, dz, qx, qy, qz].
     *
     * @param poseX The pose's translational X component.
     * @param poseY The pose's translational Y component.
     * @param poseZ The pose's translational Z component.
     * @param poseQw The pose quaternion's W component.
     * @param poseQx The pose quaternion's X component.
     * @param poseQy The pose quaternion's Y component.
     * @param poseQz The pose quaternion's Z component.
     * @param twistDx The twist's dx value.
     * @param twistDy The twist's dy value.
     * @param twistDz The twist's dz value.
     * @param twistRx The twist's rx value.
     * @param twistRy The twist's ry value.
     * @param twistRz The twist's rz value.
     * @return The new pose as a double array.
     */
    public static native double[] expPose3d(
            double poseX,
            double poseY,
            double poseZ,
            double poseQw,
            double poseQx,
            double poseQy,
            double poseQz,
            double twistDx,
            double twistDy,
            double twistDz,
            double twistRx,
            double twistRy,
            double twistRz);

    /**
     * Returns a Twist3d that maps the starting pose to the end pose.
     *
     * <p>The double array returned is of the form [dx, dy, dz, rx, ry, rz].
     *
     * @param startX The starting pose's translational X component.
     * @param startY The starting pose's translational Y component.
     * @param startZ The starting pose's translational Z component.
     * @param startQw The starting pose quaternion's W component.
     * @param startQx The starting pose quaternion's X component.
     * @param startQy The starting pose quaternion's Y component.
     * @param startQz The starting pose quaternion's Z component.
     * @param endX The ending pose's translational X component.
     * @param endY The ending pose's translational Y component.
     * @param endZ The ending pose's translational Z component.
     * @param endQw The ending pose quaternion's W component.
     * @param endQx The ending pose quaternion's X component.
     * @param endQy The ending pose quaternion's Y component.
     * @param endQz The ending pose quaternion's Z component.
     * @return The twist that maps start to end as a double array.
     */
    public static native double[] logPose3d(
            double startX,
            double startY,
            double startZ,
            double startQw,
            double startQx,
            double startQy,
            double startQz,
            double endX,
            double endY,
            double endZ,
            double endQw,
            double endQx,
            double endQy,
            double endQz);

    /**
     * Returns true if (A, B) is a stabilizable pair.
     *
     * <p>(A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if any, have
     * absolute values less than one, where an eigenvalue is uncontrollable if rank(lambda * I - A, B)
     * &lt; n where n is the number of states.
     *
     * @param states the number of states of the system.
     * @param inputs the number of inputs to the system.
     * @param A System matrix.
     * @param B Input matrix.
     * @return If the system is stabilizable.
     */
    public static native boolean isStabilizable(int states, int inputs, double[] A, double[] B);

    /**
     * Performs an inplace rank one update (or downdate) of an upper triangular Cholesky decomposition
     * matrix.
     *
     * @param mat Array of elements of the matrix to be updated.
     * @param lowerTriangular Whether mat is lower triangular.
     * @param rows How many rows there are.
     * @param vec Vector to use for the rank update.
     * @param sigma Sigma value to use for the rank update.
     */
    public static native void rankUpdate(
            double[] mat, int rows, double[] vec, double sigma, boolean lowerTriangular);

    /**
     * Solves the least-squares problem Ax=B using a QR decomposition with full pivoting.
     *
     * @param A Array of elements of the A matrix.
     * @param Arows Number of rows of the A matrix.
     * @param Acols Number of rows of the A matrix.
     * @param B Array of elements of the B matrix.
     * @param Brows Number of rows of the B matrix.
     * @param Bcols Number of rows of the B matrix.
     * @param dst Array to store solution in. If A is m-n and B is m-p, dst is n-p.
     */
    public static native void solveFullPivHouseholderQr(
            double[] A, int Arows, int Acols, double[] B, int Brows, int Bcols, double[] dst);
}
