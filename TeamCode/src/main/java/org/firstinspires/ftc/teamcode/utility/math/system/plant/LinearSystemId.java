package org.firstinspires.ftc.teamcode.utility.math.system.plant;

import org.firstinspires.ftc.teamcode.utility.math.Matrix;
import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.VecBuilder;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N1;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N2;
import org.firstinspires.ftc.teamcode.utility.math.system.LinearSystem;

public final class LinearSystemId {
    private LinearSystemId() {
        // Utility class
    }

    /**
     * Create a state-space model for a 1 DOF velocity system from its kV (volts/(unit/sec)) and kA
     * (volts/(unit/sec²). These constants cam be found using SysId. The states of the system are
     * [velocity], inputs are [voltage], and outputs are [velocity].
     *
     * <p>The distance unit you choose MUST be an SI unit (i.e. meters or radians). You can use the
     * Units class for converting between unit types.
     *
     * <p>The parameters provided by the user are from this feedforward model:
     *
     * <p>u = K_v v + K_a a
     *
     * @param kV The velocity gain, in volts/(unit/sec)
     * @param kA The acceleration gain, in volts/(unit/sec^2)
     * @return A LinearSystem representing the given characterized constants.
     * @throws IllegalArgumentException if kV &lt;= 0 or kA &lt;= 0.
     * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
     */
    public static LinearSystem<N1, N1, N1> identifyVelocitySystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
        }

        return new LinearSystem<>(
                VecBuilder.fill(-kV / kA),
                VecBuilder.fill(1.0 / kA),
                VecBuilder.fill(1.0),
                VecBuilder.fill(0.0));
    }

    /**
     * Create a state-space model for a 1 DOF position system from its kV (volts/(unit/sec)) and kA
     * (volts/(unit/sec²). These constants cam be found using SysId. The states of the system are
     * [position, velocity]ᵀ, inputs are [voltage], and outputs are [position].
     *
     * <p>The distance unit you choose MUST be an SI unit (i.e. meters or radians). You can use the
     * Units class for converting between unit types.
     *
     * <p>The parameters provided by the user are from this feedforward model:
     *
     * <p>u = K_v v + K_a a
     *
     * @param kV The velocity gain, in volts/(unit/sec)
     * @param kA The acceleration gain, in volts/(unit/sec²)
     * @return A LinearSystem representing the given characterized constants.
     * @throws IllegalArgumentException if kV &lt;= 0 or kA &lt;= 0.
     * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
     */
    public static LinearSystem<N2, N1, N1> identifyPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
        }

        return new LinearSystem<>(
                Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
                VecBuilder.fill(0.0, 1.0 / kA),
                Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0),
                VecBuilder.fill(0.0));
    }

    /**
     * Identify a differential drive drivetrain given the drivetrain's kV and kA in both linear
     * {(volts/(meter/sec), (volts/(meter/sec²))} and angular {(volts/(radian/sec)),
     * (volts/(radian/sec²))} cases. These constants can be found using SysId.
     *
     * <p>States: [[left velocity], [right velocity]]<br>
     * Inputs: [[left voltage], [right voltage]]<br>
     * Outputs: [[left velocity], [right velocity]]
     *
     * @param kVLinear The linear velocity gain in volts per (meters per second).
     * @param kALinear The linear acceleration gain in volts per (meters per second squared).
     * @param kVAngular The angular velocity gain in volts per (meters per second).
     * @param kAAngular The angular acceleration gain in volts per (meters per second squared).
     * @return A LinearSystem representing the given characterized constants.
     * @throws IllegalArgumentException if kVLinear &lt;= 0, kALinear &lt;= 0, kVAngular &lt;= 0, or
     *     kAAngular &lt;= 0.
     * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
     */
    public static LinearSystem<N2, N2, N2> identifyDrivetrainSystem(
            double kVLinear, double kALinear, double kVAngular, double kAAngular) {
        if (kVLinear <= 0.0) {
            throw new IllegalArgumentException("Kv,linear must be greater than zero.");
        }
        if (kALinear <= 0.0) {
            throw new IllegalArgumentException("Ka,linear must be greater than zero.");
        }
        if (kVAngular <= 0.0) {
            throw new IllegalArgumentException("Kv,angular must be greater than zero.");
        }
        if (kAAngular <= 0.0) {
            throw new IllegalArgumentException("Ka,angular must be greater than zero.");
        }

        final double A1 = 0.5 * -(kVLinear / kALinear + kVAngular / kAAngular);
        final double A2 = 0.5 * -(kVLinear / kALinear - kVAngular / kAAngular);
        final double B1 = 0.5 * (1.0 / kALinear + 1.0 / kAAngular);
        final double B2 = 0.5 * (1.0 / kALinear - 1.0 / kAAngular);

        return new LinearSystem<>(
                Matrix.mat(Nat.N2(), Nat.N2()).fill(A1, A2, A2, A1),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(B1, B2, B2, B1),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 0, 0, 0));
    }

    /**
     * Identify a differential drive drivetrain given the drivetrain's kV and kA in both linear
     * {(volts/(meter/sec)), (volts/(meter/sec²))} and angular {(volts/(radian/sec)),
     * (volts/(radian/sec²))} cases. This can be found using SysId.
     *
     * <p>States: [[left velocity], [right velocity]]<br>
     * Inputs: [[left voltage], [right voltage]]<br>
     * Outputs: [[left velocity], [right velocity]]
     *
     * @param kVLinear The linear velocity gain in volts per (meters per second).
     * @param kALinear The linear acceleration gain in volts per (meters per second squared).
     * @param kVAngular The angular velocity gain in volts per (radians per second).
     * @param kAAngular The angular acceleration gain in volts per (radians per second squared).
     * @param trackwidth The distance between the differential drive's left and right wheels, in
     *     meters.
     * @return A LinearSystem representing the given characterized constants.
     * @throws IllegalArgumentException if kVLinear &lt;= 0, kALinear &lt;= 0, kVAngular &lt;= 0,
     *     kAAngular &lt;= 0, or trackwidth &lt;= 0.
     * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
     */
    public static LinearSystem<N2, N2, N2> identifyDrivetrainSystem(
            double kVLinear, double kALinear, double kVAngular, double kAAngular, double trackwidth) {
        if (kVLinear <= 0.0) {
            throw new IllegalArgumentException("Kv,linear must be greater than zero.");
        }
        if (kALinear <= 0.0) {
            throw new IllegalArgumentException("Ka,linear must be greater than zero.");
        }
        if (kVAngular <= 0.0) {
            throw new IllegalArgumentException("Kv,angular must be greater than zero.");
        }
        if (kAAngular <= 0.0) {
            throw new IllegalArgumentException("Ka,angular must be greater than zero.");
        }
        if (trackwidth <= 0.0) {
            throw new IllegalArgumentException("trackwidth must be greater than zero.");
        }

        // We want to find a factor to include in Kv,angular that will convert
        // `u = Kv,angular omega` to `u = Kv,angular v`.
        //
        // v = omega r
        // omega = v/r
        // omega = 1/r v
        // omega = 1/(trackwidth/2) v
        // omega = 2/trackwidth v
        //
        // So multiplying by 2/trackwidth converts the angular gains from V/(rad/s)
        // to V/(m/s).
        return identifyDrivetrainSystem(
                kVLinear, kALinear, kVAngular * 2.0 / trackwidth, kAAngular * 2.0 / trackwidth);
    }
}
