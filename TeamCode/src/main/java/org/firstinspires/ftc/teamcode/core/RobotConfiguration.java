package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.AXON_CONTINUOUS_PWM;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.ExpansionHub;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorTypes;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;

public enum RobotConfiguration {
    IMU(
            new IMU("IMU 1")
    ),
    CONTROL_HUB(
            new ExpansionHub("Expansion Hub 173")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
    ),
    EXPANSION_HUB(
            new ExpansionHub("Expansion Hub 2")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
    ),
    DRIVE_FRONT_LEFT(
            new Motor("Front Left Drive")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_FRONT_RIGHT(
            new Motor("Front Right Drive")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_LEFT(
            new Motor("Back Left Drive")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_RIGHT(
            new Motor("Back Right Drive")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    ABSOLUTE_FRONT_LEFT(
            new AbsoluteEncoder("Front Left Encoder")
            .zero(79.3)
            .setInverted(false)
    ),
    ABSOLUTE_FRONT_RIGHT(
            new AbsoluteEncoder("Front Right Encoder")
            .zero(311.0)
            .setInverted(false)
    ),
    ABSOLUTE_BACK_LEFT(
            new AbsoluteEncoder("Back Left Encoder")
            .zero(138.7)
            .setInverted(false)
    ),
    ABSOLUTE_BACK_RIGHT(
            new AbsoluteEncoder("Back Right Encoder")
            .zero(158.0)
            .setInverted(false)
    ),
    ANGLE_FRONT_LEFT(
            new ContinuousServo("Front Left Angle")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configurePIDWrapping()
            .configurePWMRange(AXON_CONTINUOUS_PWM)
            .configurePIDF(0.009, 0.0, 0.0)
            .configureEncoder(ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder())
    ),
    ANGLE_FRONT_RIGHT(
            new ContinuousServo("Front Right Angle")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configurePIDWrapping()
            .configurePWMRange(AXON_CONTINUOUS_PWM)
            .configurePIDF(0.009, 0.0, 0.0)
            .configureEncoder(ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder())
    ),
    ANGLE_BACK_LEFT(
            new ContinuousServo("Back Left Angle")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configurePIDWrapping()
            .configurePWMRange(AXON_CONTINUOUS_PWM)
            .configurePIDF(0.009, 0.0, 0.0)
            .configureEncoder(ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder())
    ),
    ANGLE_BACK_RIGHT(
            new ContinuousServo("Back Right Angle")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configurePIDWrapping()
            .configurePWMRange(AXON_CONTINUOUS_PWM)
            .configurePIDF(0.009, 0.0, 0.0)
            .configureEncoder(ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder())
    ),
    ODOMETRY_PARALLEL_LEFT(
            new Encoder("Front Left Drive")
            .setDirection(Encoder.Direction.REVERSE)
    ),
    ODOMETRY_PARALLEL_RIGHT(
            new Encoder("Front Right Drive")
            .setDirection(Encoder.Direction.FORWARD)
    ),
    ODOMETRY_PERPENDICULAR(
            new Encoder("Back Left Drive")
            .setDirection(Encoder.Direction.FORWARD)),
    INTAKE(
            new Motor("Intake")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.OTHER)
    ),
    RAMP(
            new Servo("Ramp")
            .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD)
            .configurePWMRange(RobotConstants.AXON_PWM)
            .configureScale(0.0, 1.0)
    ),
//    SLIDE_LEFT(
//            new Motor("Slide Left")
//            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
//            .setPIDTolerance(2.0)
//            .configureDirection(DcMotorSimple.Direction.FORWARD)
//            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            .setType(MotorTypes.OTHER)
//    ),
//    SLIDE_RIGHT(
//            new Motor("Slide Right")
//            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
//            .setPIDTolerance(2.0)
//            .configureDirection(DcMotorSimple.Direction.REVERSE)
//            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//            .setType(MotorTypes.OTHER)
//    ),
    ARM_LEFT(
            new Servo("Arm Left")
            .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD)
            .configurePWMRange(RobotConstants.AXON_PWM)
            .configureScale(0.0, 1.0)
    ),
    ARM_RIGHT(
            new Servo("Arm Right")
            .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE)
            .configurePWMRange(RobotConstants.AXON_PWM)
            .configureScale(0.0, 1.0)
    ),
    CLAW_LEFT(
            new Servo("Claw Left")
            .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD)
            .configurePWMRange(RobotConstants.AXON_PWM)
            .configureScale(0.0, 1.0)
    ),
    CLAW_RIGHT(
            new Servo("Claw Right")
            .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE)
            .configurePWMRange(RobotConstants.AXON_PWM)
            .configureScale(0.0, 1.0)
    );

    private final HardwareDevice device;

    RobotConfiguration(HardwareDevice device) {
        this.device = device;
    }

    public HardwareDevice getAsHardwareDevice() {
        return device;
    }

    public IMU getAsIMU() {
        if(!(device instanceof IMU))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (IMU) device;
    }

    public Motor getAsMotor() {
        if(!(device instanceof Motor))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Motor) device;
    }

    public Servo getAsServo() {
        if(!(device instanceof Servo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Servo) device;
    }

    public ContinuousServo getAsContinuousServo() {
        if(!(device instanceof ContinuousServo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ContinuousServo) device;
    }

    public AbsoluteEncoder getAsAbsoluteEncoder() {
        if(!(device instanceof AbsoluteEncoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (AbsoluteEncoder) device;
    }

    public ExpansionHub getAsExpansionHub() {
        if(!(device instanceof ExpansionHub))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ExpansionHub) device;
    }

    public Encoder getAsEncoder() {
        if(!(device instanceof Encoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Encoder) device;
    }
    
    public Webcam getAsWebcam() {
        if(!(device instanceof Webcam))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Webcam) device;
    }
}