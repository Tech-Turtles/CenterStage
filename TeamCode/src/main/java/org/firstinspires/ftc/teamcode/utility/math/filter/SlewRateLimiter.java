package org.firstinspires.ftc.teamcode.utility.math.filter;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.utility.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a
 * {TrapezoidProfile} instead.
 */
public class SlewRateLimiter {
    private final double m_positiveRateLimit;
    private final double m_negativeRateLimit;
    private double m_prevVal;
    private double m_prevTime;

    private NanoClock clock;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.clock = NanoClock.system();

        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = clock.seconds();
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit and initial value.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     * @deprecated Use SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double
     *     initialValue) instead.
     */
    public SlewRateLimiter(double rateLimit, double initialValue) {
        this(rateLimit, -rateLimit, initialValue);
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = clock.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                MathUtil.clamp(
                        input - m_prevVal,
                        m_negativeRateLimit * elapsedTime,
                        m_positiveRateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = clock.seconds();
    }
}
