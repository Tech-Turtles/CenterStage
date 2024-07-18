package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveModulePhysicalCharacteristics;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;

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

    public static PIDFController slideController = new PIDFController(new PIDCoefficients(0.01, 0.0, 0.0001), 0, 0, 0.05);

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
    public static double TAG_UPDATE_TIME = 1.0;

    public static final String BACK_PARK = "BackPark";
    //ToDo Incorporate path constants into routine enum
    public enum AutonomousRoutine {
        PARK,
        SPIKE_PARK,
        SPIKE_PLACE_PARK,
        PLACE_PARK,
        SPIKE_PLACE_CYCLE_PARK
    }

    public static final Pose2d BLUE_BACKBOARD_START = new Pose2d(0.22, 3.36, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CENTER_START = new Pose2d(0.22, 2.15, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_AUDIENCE_START = new Pose2d(0.22, 0.82, Rotation2d.fromDegrees(180));

    public static final Pose2d RED_BACKBOARD_START = new Pose2d(3.43, 3.40, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_CENTER_START = new Pose2d(3.43, 2.15, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_AUDIENCE_START = new Pose2d(3.43, 0.86, Rotation2d.fromDegrees(0.0));


    // Mechanism Constants
    public static double WRIST_CENTER = 0.52;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -0.7;
    public enum IntakePosition {
        START(0.44),
        DRIVE(0.65),
        INTAKE(.92),
        STACK(0.83);
        private final double position;
        IntakePosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum ClawPosition {
        OPEN(0.16, .88),
        MIDDLE(0.32, .77),
        GRAB(.55, .5);
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
        START(0.77-.14, 0.77-.14),
        GRAB(0.93-.09, 0.93-.09),
        BETWEEN(0.9-.09, 0.9-.09),
        DOWN(0.9-.09, 0.9-.09),
        HOLD(0.77-.09, 0.77-.09),
        TELEOP_POS(0.8-.07, 0.8-.07),
        MIDDLE(0.6-.14, 0.6-.14),
        BACK_BOARD(0.315, 0.315),
        SPIKE(0.15, 0.15),
        SLIGHT_POS(0.78, 0.78);
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
        RIGHT_HORIZONTAL(0.25),
        MANUAL(0);
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