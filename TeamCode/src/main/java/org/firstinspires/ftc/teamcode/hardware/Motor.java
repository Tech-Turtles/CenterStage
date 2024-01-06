package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.swerve.configuration.PIDFConfig;
import org.firstinspires.ftc.teamcode.utility.math.controller.PIDController;

public class Motor extends HardwareDevice {
    private DcMotorEx device;
    private DcMotor.Direction direction = Direction.FORWARD;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private MotorTypes type = MotorTypes.OTHER;
    private final PIDController controller = new PIDController(0.0, 0.0, 0.0, 0.01);
    private double lastPower;
    private int offset;

    public Motor(String configName) {
        super(configName, DcMotorEx.class);
    }

    public Motor(String configName, PIDFConfig pidfConfig) {
        this(configName);
        configurePID(pidfConfig.p, pidfConfig.i, pidfConfig.d);
        setPIDTolerance(1.0);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }
        lastPower = 2;
        this.device = (DcMotorEx) device;
        this.device.setZeroPowerBehavior(zeroPowerBehavior);
        this.device.setDirection(direction);
        this.device.setMode(runMode);
        setStatus(HardwareStatus.SUCCESS);
    }
    public Motor configurePID(double p, double i, double d) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.reset();
        return this;
    }

    public Motor setPIDTolerance(double positionTolerance) {
        controller.setTolerance(positionTolerance);
        return this;
    }

    public Motor configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public Motor configureDirection(DcMotor.Direction direction) {
        this.direction = direction;
        return this;
    }

    public Motor configureRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
        return this;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    public DcMotor.RunMode getRunMode() {
        return runMode;
    }

    public MotorTypes getType() {
        return type;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.device.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDirection(DcMotor.Direction direction) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.direction = direction;
        this.device.setDirection(direction);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.runMode = runMode;
        this.device.setMode(runMode);
    }

    public Motor setType(MotorTypes type) {
        this.type = type;
        return this;
    }

    public double getPower() {
        if (getStatus().equals(HardwareStatus.MISSING)) return 0;
        return lastPower;
    }

    public void setPower(double power) {
        if(getStatus().equals(HardwareStatus.MISSING) || Math.abs(power - lastPower) < RobotConstants.MOTOR_CACHE_TOLERANCE) return;
        lastPower = power;
        device.setPower(power);
    }

    public void setReference(double setpoint) {
        setReference(setpoint, getEncoderValue(),0.0);
    }

    public void setReference(double setpoint, double measurement) {
        setReference(setpoint, measurement, 0.0);
    }

    public void setReference(double setpoint, double measurement, double feedforward) {
        if (getStatus().equals(HardwareStatus.MISSING)) return;
        setPower(controller.calculate(measurement, setpoint) + feedforward);
    }

    public void setEncoderPositionOffset(int offset) {
        this.offset += offset;
    }

    public int getRawEncoderValue() {
        if (getStatus().equals(HardwareStatus.MISSING)) return 0;
        return device.getCurrentPosition();
    }

    public int getEncoderValue() {
        if (getStatus().equals(HardwareStatus.MISSING)) return 0;
        return device.getCurrentPosition() - offset;
    }

    public double getVelocity() {
        if (getStatus().equals(HardwareStatus.MISSING)) return 0.0;
        return device.getVelocity();
    }

    public double getCurrent() {
        if (getStatus().equals(HardwareStatus.MISSING)) return 0.0;
        return device.getCurrent(CurrentUnit.AMPS);
    }
}