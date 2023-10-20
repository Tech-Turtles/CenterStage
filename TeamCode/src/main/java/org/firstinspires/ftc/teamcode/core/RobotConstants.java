package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
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
            new PwmControl.PwmRange(510, 2490, 5000);

    public static final int DRIVE_TICKS_PER_METER = 1230;

    //ToDo Incorporate proper maximum speed limiting on swerve

    // units m/s
    public static final double SWERVE_MAX_SPEED = 2.2;
    // units m/s
    public static final double SWERVE_PRECISION_SPEED = 0.5;
    // seconds; delay between heading correction and last heading input,
    // allowing robot to continue its momentum before correcting
    public static double HEADING_TIME = 0.1;

    public static final String BACK_PARK = "Back Park";
    //ToDo Incorporate path constants into routine enum
    public enum AutonomousRoutine {
        PARK,
        SPIKE_PARK,
        SPIKE_PLACE_PARK,
        PLACE_PARK
    }

    // Mechanism Constants
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -0.7;
    public enum IntakePosition {
        START(0.0),
        DRIVE(0.2),
        INTAKE(0.63),
        STACK(0.55);
        private final double position;
        IntakePosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum ClawPosition {
        OPEN(0.0, 0.0),
        CLOSE(0.0, 0.0);
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
        START(0.0, 0.0),
        GRAB(0.0, 0.0),
        HOLD(0.0, 0.0),
        BACK_BOARD(0.0, 0.0),
        SPIKE(0.0, 0.0);
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
    public static final double DEADZONE = 0.2;
}