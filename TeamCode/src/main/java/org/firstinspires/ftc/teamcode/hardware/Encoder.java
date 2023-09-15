package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;

public class Encoder extends HardwareDevice {

    private final static int CPS_STEP = 0x10000;
    private DcMotorEx device;
    private NanoClock clock;

    private Direction direction = Direction.FORWARD;

    private int lastPosition;
    private double velocityEstimate;
    private double lastUpdateTime;

    public Encoder(String configName) {
        super(configName, DcMotorEx.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.device = (DcMotorEx) device;
        this.clock = NanoClock.system();

        this.lastPosition = 0;
        this.velocityEstimate = 0.0;
        this.lastUpdateTime = clock.seconds();

        setStatus(HardwareStatus.SUCCESS);
    }

    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public Encoder setDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public Direction getDirection() {
        return direction;
    }

    public int getCurrentPosition() {
        if(getStatus().equals(HardwareStatus.MISSING)) return 0;

        int currentPosition = device.getCurrentPosition() * direction.getMultiplier();
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        if(getStatus().equals(HardwareStatus.MISSING)) return 0.0;

        return device.getVelocity() * direction.getMultiplier();
    }

    public double getCorrectedVelocity() {
        return inverseOverflow(getRawVelocity(), velocityEstimate);
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }
}