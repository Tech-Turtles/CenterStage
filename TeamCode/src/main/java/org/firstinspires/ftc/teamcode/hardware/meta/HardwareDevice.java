package org.firstinspires.ftc.teamcode.hardware.meta;

public abstract class HardwareDevice {
    private final String configName;
    private final Class<? extends com.qualcomm.robotcore.hardware.HardwareDevice> deviceClass;
    private HardwareStatus status = HardwareStatus.MISSING;

    public HardwareDevice(String configName, Class<? extends com.qualcomm.robotcore.hardware.HardwareDevice> clazz) {
        this.configName = configName;
        this.deviceClass = clazz;
    }

    protected void setStatus(HardwareStatus status) {
        this.status = status;
    }

    public HardwareStatus getStatus() {
        return status;
    }

    public String getConfigName() {
        return configName;
    }

    public Class<? extends com.qualcomm.robotcore.hardware.HardwareDevice> getDeviceClass() {
        return deviceClass;
    }

    public abstract void initialize(Object device);
}