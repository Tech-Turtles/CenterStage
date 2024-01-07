package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveModulePhysicalCharacteristics;

@Config
public final class RobotConstants {
    public static final SwerveModulePhysicalCharacteristics SWERVE_MODULE_PHYSICAL_CHARACTERISTICS =
            new SwerveModulePhysicalCharacteristics(
            10, 1.0, 0.072, 1.1,
            12.0, 20, 20, 0.25,
            0.25, 1, 1, 0.0
    );

    public static final PwmControl.PwmRange AXON_CONTINUOUS_PWM =
            new PwmControl.PwmRange(500, 2500, 5000);

    public static final PwmControl.PwmRange AXON_PWM =
            new PwmControl.PwmRange(520, 2480, 5000);

    public static PIDFController slideController = new PIDFController(new PIDCoefficients(0.04, 0.0, 0.0), 0, 0, 0.1);

    public static final int DRIVE_TICKS_PER_METER = 1230;
    public static final double MOTOR_CACHE_TOLERANCE = 0.02;

    //ToDo Incorporate proper maximum speed limiting on swerve

    // units m/s
    public static final double SWERVE_MAX_SPEED = 2.2;
    // units m/s
    public static final double SWERVE_PRECISION_SPEED = 0.5;
    // seconds; delay between heading correction and last heading input,
    // allowing robot to continue its momentum before correcting
    public static double HEADING_TIME = 0.1;

    public static final String BACK_PARK = "BackPark";
    //ToDo Incorporate path constants into routine enum
    public enum AutonomousRoutine {
        PARK,
        SPIKE_PARK,
        SPIKE_PLACE_PARK,
        PLACE_PARK
    }

    // Mechanism Constants
    public static double WRIST_CENTER = 0.52;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -0.7;
    public enum IntakePosition {
        START(0.3),
        DRIVE(0.55),
        INTAKE(0.89),
        STACK(0.8);
        private final double position;
        IntakePosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum ClawPosition {
        OPEN(0.1, .88),
        MIDDLE(0.32, .77),
        GRAB(.51, .47);
        private final double leftPos, rightPos;
        ClawPosition(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }

        public double getLeftPos() {
            return leftPos;
        }

        public double getRightPos() {
            return rightPos;
        }
    }

    public enum ClawOrder {
        BOTH,
        LEFT,
        RIGHT
    }

    public enum ArmPosition {
        START(0.77, 0.77),
        GRAB(0.94, 0.94),
        BETWEEN(0.9, 0.9),
        DOWN(0.9, 0.9),
        HOLD(0.77, 0.77),
        TELEOP_POS(0.8, 0.8),
        BACK_BOARD(0.52, 0.52),
        SPIKE(0.47, 0.47);
        private final double leftPos, rightPos;
        ArmPosition(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }

        public double getLeftPos() {
            return leftPos;
        }

        public double getRightPos() {
            return rightPos;
        }
    }

    public enum WristPosition {
        START(0.52),
        VERTICAL(0.52),
        LEFT_HORIZONTAL(0.78),
        RIGHT_HORIZONTAL(0.25);
        private final double position;
        WristPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }
    public static final double DEADZONE = 0.2;
}