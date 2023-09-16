package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.SWERVE_MODULE_PHYSICAL_CHARACTERISTICS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Controller;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.configuration.PIDFConfig;
import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveControllerConfiguration;
import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveDriveConfiguration;
import org.firstinspires.ftc.teamcode.swerve.configuration.SwerveModuleConfiguration;
import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;

//ToDo Fix Angle Offset so it is passed to the SwerveModuleConfiguration, not on the encoder constructor
//ToDo Make SwerveModuleConfiguration parameters constants in RobotConstants
public class RobotHardware extends OpMode {

    // Hub & hub sensor objects.
    public VoltageSensor batteryVoltageSensor;
    // Timer to keep track of the period & period mean times.
    public final ElapsedTimer period = new ElapsedTimer();
    // Controller objects that act as a more intuitive wrapper for the FTC gamepad class.
    public Controller primary, secondary;
    private FtcDashboard dashboard;
    public TelemetryPacket packet;
    //ToDo Save last position measured by robot, so it can transfer between OpModes
    public static Pose2d lastPosition = new Pose2d(0,0,0);
    protected SwerveControllerConfiguration swerveControllerConfiguration;
    public SwerveDrive swerveDrive;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        packet = new TelemetryPacket();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            try {
                device.initialize(hardwareMap.get(device.getDeviceClass(), device.getConfigName()));
            } catch (IllegalArgumentException ignore) {}
        }

        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        RobotConfiguration.EXPANSION_HUB.getAsExpansionHub().clearBulkCache();

        SwerveModuleConfiguration front_left = new SwerveModuleConfiguration(
                RobotConfiguration.DRIVE_FRONT_LEFT.getAsMotor(),
                RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo(),
                RobotConfiguration.ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder(),
                0.1778, 0.1778, new PIDFConfig(0.006,0.0),
                new PIDFConfig(0.08, 0.0), 2.2, SWERVE_MODULE_PHYSICAL_CHARACTERISTICS, "FrontLeft"
        );

        SwerveModuleConfiguration front_right = new SwerveModuleConfiguration(
                RobotConfiguration.DRIVE_FRONT_RIGHT.getAsMotor(),
                RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo(),
                RobotConfiguration.ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder(),
                0.1778, -0.1778, new PIDFConfig(0.006,0.0),
                new PIDFConfig(0.08, 0.0), 2.2, SWERVE_MODULE_PHYSICAL_CHARACTERISTICS, "FrontRight"
        );

        SwerveModuleConfiguration back_left = new SwerveModuleConfiguration(
                RobotConfiguration.DRIVE_BACK_LEFT.getAsMotor(),
                RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo(),
                RobotConfiguration.ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder(),
                -0.1778, 0.1778, new PIDFConfig(0.008,0.0),
                new PIDFConfig(0.08, 0.0), 2.2, SWERVE_MODULE_PHYSICAL_CHARACTERISTICS, "BackLeft"
        );

        SwerveModuleConfiguration back_right = new SwerveModuleConfiguration(
                RobotConfiguration.DRIVE_BACK_RIGHT.getAsMotor(),
                RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo(),
                RobotConfiguration.ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder(),
                -0.1778, -0.1778, new PIDFConfig(0.009,0.0),
                new PIDFConfig(0.08, 0.0), 2.2, SWERVE_MODULE_PHYSICAL_CHARACTERISTICS, "BackRight"
        );

        SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[]{front_left, back_left, back_right, front_right},
                RobotConfiguration.IMU.getAsIMU(), 2.2, false);

        swerveControllerConfiguration = new SwerveControllerConfiguration(
                swerveDriveConfiguration, new PIDFConfig(0.08, 0.0));

        swerveDrive = new SwerveDrive(swerveDriveConfiguration, swerveControllerConfiguration);


        //ToDo Add Photon once it is updated to work on SDK 9.0
//        PhotonCore.enable();

        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);

        period.reset();
    }

    @Override
    public void init_loop() {
        super.init_loop();

        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        RobotConfiguration.EXPANSION_HUB.getAsExpansionHub().clearBulkCache();

        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            if(device.getStatus().equals(HardwareStatus.MISSING)) {
                telemetry.addData(robotConfiguration.name() + " Missing; Config Name: ", device.getConfigName());
                if(packet != null)
                    packet.put(robotConfiguration.name() + " Missing; Config Name: ", device.getConfigName());
            }
        }

        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }

        primary.update();
        secondary.update();

        period.updatePeriodTime();
    }

    @Override
    public void start() {
        super.start();

        if(packet != null)
            packet = new TelemetryPacket();


        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        RobotConfiguration.EXPANSION_HUB.getAsExpansionHub().clearBulkCache();

        period.reset();
    }

    @Override
    public void loop() {
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet.clearLines();
        }

        telemetry.addData("Period Average","%.3f sec", period.getAveragePeriodSec());

        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        RobotConfiguration.EXPANSION_HUB.getAsExpansionHub().clearBulkCache();

        primary.update();
        secondary.update();

        period.updatePeriodTime();
    }

    @Override
    public void stop() {
        if(packet != null)
            dashboard.sendTelemetryPacket(packet);
        packet = null;
        dashboard = null;

        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        RobotConfiguration.EXPANSION_HUB.getAsExpansionHub().clearBulkCache();

        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            if(device instanceof Motor)
                ((Motor) device).setPower(0.0);
            else if(device instanceof ContinuousServo)
                ((ContinuousServo) device).setPower(0.0);
            else if(device instanceof Webcam)
                ((Webcam) device).stop();
        }
    }

    public static String setFontColor(String color, String text) {
        return "<font color=\""+color+"\">"+text+"</font>";
    }

    public static String setHeader(int headerNumber, String text) {
        return "<h"+headerNumber+">"+text+"</h"+headerNumber+">";
    }
}