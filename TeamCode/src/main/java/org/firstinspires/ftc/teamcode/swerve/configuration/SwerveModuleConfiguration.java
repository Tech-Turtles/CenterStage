package org.firstinspires.ftc.teamcode.swerve.configuration;

import static org.firstinspires.ftc.teamcode.swerve.SwerveMath.calculateDegreesPerSteeringRotation;
import static org.firstinspires.ftc.teamcode.swerve.SwerveMath.calculateMaxAcceleration;
import static org.firstinspires.ftc.teamcode.swerve.SwerveMath.calculateMetersPerRotation;

import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.utility.math.controller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

/**
 * Swerve Module configuration class which is used to configure SwerveModule.
 */
public class SwerveModuleConfiguration
{

    /**
     * Angle offset in degrees for the Swerve Module.
     */
    public final double                              angleOffset;
    /**
     * Whether the absolute encoder is inverted.
     */
    public final boolean                             absoluteEncoderInverted;
    /**
     * State of inversion of the drive motor.
     */
    public final boolean                             driveMotorInverted;
    /**
     * State of inversion of the angle motor.
     */
    public final boolean                             angleMotorInverted;
    /**
     * Maximum robot speed in meters per second.
     */
    public       double                              maxSpeed;
    /**
     * PIDF configuration options for the angle motor closed-loop PID controller.
     */
    public       PIDFConfig                          anglePIDF;
    /**
     * PIDF configuration options for the drive motor closed-loop PID controller.
     */
    public       PIDFConfig                          velocityPIDF;
    /**
     * Angle volt-meter-per-second.
     */
    public       double                              moduleSteerFFCL;
    /**
     * The integrated encoder pulse per revolution.
     */
    public       double                              angleMotorEncoderPulsePerRevolution = 0;
    /**
     * Swerve module location relative to the robot.
     */
    public Translation2d                              moduleLocation;
    /**
     * Physical characteristics of the swerve module.
     */
    public       SwerveModulePhysicalCharacteristics physicalCharacteristics;
    /**
     * The drive motor and angle motor of this swerve module.
     */
    public Motor driveMotor;
    /**
     * The drive motor and angle motor of this swerve module.
     */
    public ContinuousServo angleServo;
    /**
     * The Absolute Encoder for the swerve module.
     */
    public AbsoluteEncoder absoluteEncoder;
    /**
     * Name for the swerve module for telemetry.
     */
    public String                name;

    /**
     * Construct a configuration object for swerve modules.
     *
     * @param driveMotor                          Drive
     * @param angleServo                          Angle
     * @param absoluteEncoder                     Absolute encoder {@link AbsoluteEncoder}.
     * @param angleOffset                         Absolute angle offset to 0.
     * @param absoluteEncoderInverted             Absolute encoder inverted.
     * @param angleMotorInverted                  State of inversion of the angle motor.
     * @param driveMotorInverted                  Drive motor inverted.
     * @param xMeters                             Module location in meters from the center horizontally.
     * @param yMeters                             Module location in meters from center vertically.
     * @param anglePIDF                           Angle PIDF configuration.
     * @param velocityPIDF                        Velocity PIDF configuration.
     * @param maxSpeed                            Maximum speed in meters per second.
     * @param physicalCharacteristics             Physical characteristics of the swerve module.
     * @param angleMotorEncoderPulsePerRevolution The encoder pulse per revolution for the angle motor encoder.
     * @param name                                The name for the swerve module.
     */
    public SwerveModuleConfiguration(
            Motor driveMotor,
            ContinuousServo angleServo,
            AbsoluteEncoder absoluteEncoder,
            double angleOffset,
            double xMeters,
            double yMeters,
            PIDFConfig anglePIDF,
            PIDFConfig velocityPIDF,
            double maxSpeed,
            SwerveModulePhysicalCharacteristics physicalCharacteristics,
            boolean absoluteEncoderInverted,
            boolean driveMotorInverted,
            boolean angleMotorInverted,
            double angleMotorEncoderPulsePerRevolution,
            String name)
    {
        this.driveMotor = driveMotor;
        this.angleServo = angleServo;
        this.absoluteEncoder = absoluteEncoder;
        this.angleOffset = angleOffset;
        this.absoluteEncoderInverted = absoluteEncoderInverted;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.moduleLocation = new Translation2d(xMeters, yMeters);
        this.anglePIDF = anglePIDF;
        this.velocityPIDF = velocityPIDF;
        this.maxSpeed = maxSpeed;
        this.moduleSteerFFCL = physicalCharacteristics.moduleSteerFFCL;
        this.physicalCharacteristics = physicalCharacteristics;
        this.angleMotorEncoderPulsePerRevolution = angleMotorEncoderPulsePerRevolution;
        this.name = name;
    }

    /**
     * Construct a configuration object for swerve modules. Assumes the absolute encoder and drive motor are not
     * inverted.
     *
     * @param driveMotor              Drive
     * @param angleServo              Angle
     * @param absoluteEncoder         Absolute encoder {@link AbsoluteEncoder}.
     * @param angleOffset             Absolute angle offset to 0.
     * @param xMeters                 Module location in meters from the center horizontally.
     * @param yMeters                 Module location in meters from center vertically.
     * @param anglePIDF               Angle PIDF configuration.
     * @param velocityPIDF            Velocity PIDF configuration.
     * @param maxSpeed                Maximum robot speed in meters per second.
     * @param physicalCharacteristics Physical characteristics of the swerve module.
     * @param name                    Name for the module.
     */
    public SwerveModuleConfiguration(
            Motor driveMotor,
            ContinuousServo angleServo,
            AbsoluteEncoder absoluteEncoder,
            double angleOffset,
            double xMeters,
            double yMeters,
            PIDFConfig anglePIDF,
            PIDFConfig velocityPIDF,
            double maxSpeed,
            SwerveModulePhysicalCharacteristics physicalCharacteristics,
            String name)
    {
        this(
                driveMotor,
                angleServo,
                absoluteEncoder,
                angleOffset,
                xMeters,
                yMeters,
                anglePIDF,
                velocityPIDF,
                maxSpeed,
                physicalCharacteristics,
                false,
                false,
                false,
                physicalCharacteristics.angleEncoderPulsePerRotation,
                name);
    }

    public SwerveModuleConfiguration(
            Motor driveMotor,
            ContinuousServo angleServo,
            AbsoluteEncoder absoluteEncoder,
            double xMeters,
            double yMeters,
            PIDFConfig anglePIDF,
            PIDFConfig velocityPIDF,
            double maxSpeed,
            SwerveModulePhysicalCharacteristics physicalCharacteristics,
            String name)
    {
        this(
                driveMotor,
                angleServo,
                absoluteEncoder,
                0.0,
                xMeters,
                yMeters,
                anglePIDF,
                velocityPIDF,
                maxSpeed,
                physicalCharacteristics,
                false,
                false,
                false,
                physicalCharacteristics.angleEncoderPulsePerRotation,
                name);
    }

    /**
     * Create the drive feedforward for swerve modules.
     *
     * @return Drive feedforward for drive motor on a swerve module.
     */
    public SimpleMotorFeedforward createDriveFeedforward()
    {
        double kv = physicalCharacteristics.optimalVoltage / maxSpeed;
        /// ^ Volt-seconds per meter (max voltage divided by max speed)
        double ka =
                physicalCharacteristics.optimalVoltage
                        / calculateMaxAcceleration(physicalCharacteristics.wheelGripCoefficientOfFriction);
        /// ^ Volt-seconds^2 per meter (max voltage divided by max accel)
        return new SimpleMotorFeedforward(0, kv, ka);
    }

    /**
     * Get the encoder conversion for position encoders.
     *
     * @param isDriveMotor For the drive motor.
     * @return Position encoder conversion factor.
     */
    public double getPositionEncoderConversion(boolean isDriveMotor)
    {
        return isDriveMotor
                ? calculateMetersPerRotation(
                physicalCharacteristics.wheelDiameter,
                physicalCharacteristics.driveGearRatio,
                angleMotorEncoderPulsePerRevolution)
                : calculateDegreesPerSteeringRotation(
                physicalCharacteristics.angleGearRatio,
                angleMotorEncoderPulsePerRevolution);
    }
}
