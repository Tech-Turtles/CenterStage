package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveModulePhysicalCharacteristics;

@Config
public final class RobotConstants {
    public static final SwerveModulePhysicalCharacteristics SWERVE_MODULE_PHYSICAL_CHARACTERISTICS =
            new SwerveModulePhysicalCharacteristics(
            8.181818, 1.0, 0.072, 1.1,
            12.0, 20, 20, 0.25,
            0.25, 1, 1, 0.0
    );

    public static final PwmControl.PwmRange AXON_CONTINUOUS_PWM =
            new PwmControl.PwmRange(500, 2500, 5000);

    public static final PwmControl.PwmRange AXON_PWM =
            new PwmControl.PwmRange(510, 2490, 5000);

    public static final int DRIVE_TICKS_PER_METER = 1230;
    // units m/s
    public static final double SWERVE_MAX_SPEED = 2.2;
    // units m/s
    public static final double SWERVE_PRECISION_SPEED = 1.0;

    // Mechanism Constants
    public static double INTAKE_SPEED = 0.7;
    public static double OUTTAKE_SPEED = -0.7;
    public enum IntakePosition {
        START(0.5),
        DRIVE(0.4),
        INTAKE(0.15),
        STACK(0.25);
        private final double position;
        IntakePosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }
    public static final double DEADZONE = 0.5;
}
