package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;

public class Servo extends HardwareDevice {
    private ServoImplEx device;
    private double min = 0.0, max = 1.0;
    private Direction direction = Direction.FORWARD;
    private PwmControl.PwmRange pwmRange = null;
    public Servo(String configName) {
        super(configName, ServoImplEx.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }
        this.device = (ServoImplEx) device;
        this.device.setDirection(direction);
        this.device.scaleRange(min, max);
        if(pwmRange != null) {
            this.device.setPwmEnable();
            this.device.setPwmRange(pwmRange);
        }
        setStatus(HardwareStatus.SUCCESS);
    }
    public Servo configureDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public Servo configureScale(double min, double max) {
        this.min = min;
        this.max = max;
        return this;
    }

    public Servo configurePWMRange(PwmControl.PwmRange pwmRange) {
        this.pwmRange = pwmRange;
        return this;
    }

    public void setPosition(double position) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        device.setPosition(position);
    }

    public void setDirection(Direction direction) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.direction = direction;
        device.setDirection(direction);
    }

    public void setScale(double min, double max) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.min = min;
        this.max = max;
        device.scaleRange(min, max);
    }

    public void setPWMRange(PwmControl.PwmRange pwmRange) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.pwmRange = pwmRange;
        device.setPwmRange(pwmRange);
    }

    public void disablePWM() {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        device.setPwmDisable();
    }
}