package org.firstinspires.ftc.teamcode.utility.math.trajectory;

import java.util.Objects;

/**
 * A trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
 * reference obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * TrapezoidProfile.Constraints constraints =
 *   new TrapezoidProfile.Constraints(kMaxV, kMaxA);
 * TrapezoidProfile.State previousProfiledReference =
 *   new TrapezoidProfile.State(initialReference, 0.0);
 * TrapezoidProfile profile = new TrapezoidProfile(constraints);
 * </code></pre>
 *
 * <p>Run on update:
 *
 * <pre><code>
 * previousProfiledReference =
 * profile.calculate(timeSincePreviousUpdate, unprofiledReference, previousProfiledReference);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `isFinished()`.
 */
public class TrapezoidProfile {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private int m_direction;

    private final Constraints m_constraints;
    private State m_current;

    private double m_endAccel;
    private double m_endFullSpeed;
    private double m_endDeccel;

    public static class Constraints {
        public final double maxVelocity;

        public final double maxAcceleration;

        /**
         * Construct constraints for a TrapezoidProfile.
         *
         * @param maxVelocity maximum velocity
         * @param maxAcceleration maximum acceleration
         */
        public Constraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
    }

    public static class State {
        public double position;

        public double velocity;

        public State() {}

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return this.position == rhs.position && this.velocity == rhs.velocity;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(position, velocity);
        }
    }

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     */
    public TrapezoidProfile(Constraints constraints) {
        m_constraints = constraints;
    }
    /**
     * Calculate the correct position and velocity for the profile at a time t where the beginning of
     * the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     * @param goal The desired state when the profile is complete.
     * @param current The current state.
     * @return The position and velocity of the profile at time t.
     */
    public State calculate(double t, State goal, State current) {
        m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
        m_current = direct(current);
        goal = direct(goal);

        if (m_current.velocity > m_constraints.maxVelocity) {
            m_current.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

        double cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist =
                cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
        double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

        double fullSpeedDist =
                fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        m_endAccel = accelerationTime - cutoffBegin;
        m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
        m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
        State result = new State(m_current.position, m_current.velocity);

        if (t < m_endAccel) {
            result.velocity += t * m_constraints.maxAcceleration;
            result.position += (m_current.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
        } else if (t < m_endFullSpeed) {
            result.velocity = m_constraints.maxVelocity;
            result.position +=
                    (m_current.velocity + m_endAccel * m_constraints.maxAcceleration / 2.0) * m_endAccel
                            + m_constraints.maxVelocity * (t - m_endAccel);
        } else if (t <= m_endDeccel) {
            result.velocity = goal.velocity + (m_endDeccel - t) * m_constraints.maxAcceleration;
            double timeLeft = m_endDeccel - t;
            result.position =
                    goal.position
                            - (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) * timeLeft;
        } else {
            result = goal;
        }

        return direct(result);
    }

    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param target The target distance.
     * @return The time left until a target distance in the profile is reached.
     */
    public double timeLeftUntil(double target) {
        double position = m_current.position * m_direction;
        double velocity = m_current.velocity * m_direction;

        double endAccel = m_endAccel * m_direction;
        double endFullSpeed = m_endFullSpeed * m_direction - endAccel;

        if (target < position) {
            endAccel = -endAccel;
            endFullSpeed = -endFullSpeed;
            velocity = -velocity;
        }

        endAccel = Math.max(endAccel, 0);
        endFullSpeed = Math.max(endFullSpeed, 0);

        final double acceleration = m_constraints.maxAcceleration;
        final double decceleration = -m_constraints.maxAcceleration;

        double distToTarget = Math.abs(target - position);
        if (distToTarget < 1e-6) {
            return 0;
        }

        double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

        double deccelVelocity;
        if (endAccel > 0) {
            deccelVelocity = Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accelDist));
        } else {
            deccelVelocity = velocity;
        }

        double fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;
        double deccelDist;

        if (accelDist > distToTarget) {
            accelDist = distToTarget;
            fullSpeedDist = 0;
            deccelDist = 0;
        } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist;
            deccelDist = 0;
        } else {
            deccelDist = distToTarget - fullSpeedDist - accelDist;
        }

        double accelTime =
                (-velocity + Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accelDist)))
                        / acceleration;

        double deccelTime =
                (-deccelVelocity
                        + Math.sqrt(
                        Math.abs(deccelVelocity * deccelVelocity + 2 * decceleration * deccelDist)))
                        / decceleration;

        double fullSpeedTime = fullSpeedDist / m_constraints.maxVelocity;

        return accelTime + fullSpeedTime + deccelTime;
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal.
     */
    public double totalTime() {
        return m_endDeccel;
    }

    /**
     * Returns true if the profile has reached the goal.
     *
     * <p>The profile has reached the goal if the time since the profile started has exceeded the
     * profile's total time.
     *
     * @param t The time since the beginning of the profile.
     * @return True if the profile has reached the goal.
     */
    public boolean isFinished(double t) {
        return t >= totalTime();
    }

    /**
     * Returns true if the profile inverted.
     *
     * <p>The profile is inverted if goal position is less than the initial position.
     *
     * @param initial The initial state (usually the current state).
     * @param goal The desired state when the profile is complete.
     */
    private static boolean shouldFlipAcceleration(State initial, State goal) {
        return initial.position > goal.position;
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * m_direction;
        result.velocity = result.velocity * m_direction;
        return result;
    }
}