package org.firstinspires.ftc.teamcode.swerve.configuration;

import org.firstinspires.ftc.teamcode.utility.math.controller.PIDController;

/**
 * Hold the PIDF and Integral Zone values for a PID.
 */
public class PIDFConfig
{

    /**
     * Proportional Gain for PID.
     */
    public double p;
    /**
     * Integral Gain for PID.
     */
    public double i;
    /**
     * Derivative Gain for PID.
     */
    public double d;
    /**
     * Feedforward value for PID.
     */
    public double f;
    /**
     * Integral zone of the PID.
     */
    public double iz;

    /**
     * Minimum value.
     */
    public double min = -1;
    /**
     * Maximum value.
     */
    public double max = 1;

    /**
     * PIDF Config constructor to contain the values.
     *
     * @param p  P gain.
     * @param i  I gain.
     * @param d  D gain.
     * @param f  F gain.
     * @param iz Intergral zone.
     */
    public PIDFConfig(double p, double i, double d, double f, double iz)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.iz = iz;
    }

    /**
     * PIDF Config constructor to contain the values.
     *
     * @param p P gain.
     * @param i I gain.
     * @param d D gain.
     * @param f F gain.
     */
    public PIDFConfig(double p, double i, double d, double f)
    {
        this(p, i, d, f, 0);
    }

    /**
     * PIDF Config constructor to contain the values.
     *
     * @param p P gain.
     * @param i I gain.
     * @param d D gain.
     */
    public PIDFConfig(double p, double i, double d)
    {
        this(p, i, d, 0, 0);
    }

    /**
     * PIDF Config constructor to contain the values.
     *
     * @param p P gain.
     * @param d D gain.
     */
    public PIDFConfig(double p, double d)
    {
        this(p, 0, d, 0, 0);
    }

    /**
     * Create a PIDController from the PID values.
     *
     * @return PIDController.
     */
    public PIDController createPIDController()
    {
        return new PIDController(p, i, d);
    }
}