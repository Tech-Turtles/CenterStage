package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;

//ToDo Normalize getCurentPosition()'s angle due to subtracting the offset when returning
public class AbsoluteEncoder extends HardwareDevice {

    public static final double DEFAULT_RANGE = 3.3;
    private AnalogInput device;
    private double analogRange;
    private double offset;
    private boolean inverted;
    private NanoClock clock;
    private double lastPosition;
    private double lastUpdateTime;

    public AbsoluteEncoder(String configName) {
        super(configName, AnalogInput.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.device = (AnalogInput) device;

        analogRange = DEFAULT_RANGE;
        offset = 0;
        inverted = false;

        this.clock = NanoClock.system();

        this.lastPosition = 0.0;
        this.lastUpdateTime = clock.seconds();

        setStatus(HardwareStatus.SUCCESS);
    }

    public AbsoluteEncoder setAnalogRange(double range) {
        this.analogRange = range;
        return this;
    }

    public AbsoluteEncoder zero(double offset){
        this.offset = offset;
        return this;
    }
    public AbsoluteEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }

    public boolean getInverted() {
        return inverted;
    }

    public double getCurrentPosition() {
        if(getStatus().equals(HardwareStatus.MISSING)) return 0.0;

        double pos = (inverted
                ? (device.getVoltage() / analogRange)
                : (1 - (device.getVoltage() / analogRange))) * 360.0;

        return pos - offset;
    }

    public double getVelocity() {
        double currentPosition = getCurrentPosition();
        double currentTime = clock.seconds();
        double dt = currentTime - lastUpdateTime;
        double velocityEstimate = (currentPosition - lastPosition) / dt;
        lastPosition = currentPosition;
        lastUpdateTime = currentTime;
        return velocityEstimate;
    }
}