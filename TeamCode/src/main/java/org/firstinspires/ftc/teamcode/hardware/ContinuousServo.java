package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.utility.math.controller.PIDController;

public class ContinuousServo extends HardwareDevice {
    private CRServoImplEx device;
    private Direction direction = Direction.FORWARD;
    private PwmControl.PwmRange pwmRange = null;
    private final PIDController controller = new PIDController(0.0, 0.0, 0.0, 0.01);
    private double feedforward;
    private NanoClock clock;
    private AbsoluteEncoder encoder = null;
    private double velocityEstimate;
    private double lastPosition;
    private double lastUpdateTime;

    public ContinuousServo(String configName) {
        super(configName, CRServoImplEx.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.controller.setTolerance(0.5);

        this.device = (CRServoImplEx) device;
        this.device.setDirection(direction);

        if(pwmRange != null)
            this.device.setPwmRange(pwmRange);

        this.clock = NanoClock.system();

        this.lastPosition = 0.0;
        this.lastUpdateTime = clock.seconds();

        setStatus(HardwareStatus.SUCCESS);
    }

    public ContinuousServo configurePIDF(double p, double i, double d) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.reset();
        return this;
    }
    public ContinuousServo configurePIDF(double p, double i, double d, double ff) {
        configurePIDF(p,i,d);
        this.feedforward = ff;
        return this;
    }

    public ContinuousServo configurePIDWrapping() {
        return this.configurePIDWrapping(0.0, 360.0);
    }

    public ContinuousServo configurePIDWrapping(double minInput, double maxInput) {
        controller.enableContinuousInput(minInput, maxInput);
        return this;
    }

    public ContinuousServo configureDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public ContinuousServo configurePWMRange(PwmControl.PwmRange range) {
        this.pwmRange = range;
        return this;
    }

    public ContinuousServo configureEncoder(AbsoluteEncoder encoder) {
        this.encoder = encoder;
        return this;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.direction = direction;
        device.setDirection(direction);
    }

    public void setPWMRange(PwmControl.PwmRange range) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        this.pwmRange = range;
        device.setPwmRange(range);
    }

    public void setPower(double percentOutput) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;
        device.setPower(percentOutput);
    }

    public void setReference(double setpoint) {
        setReference(setpoint, feedforward, getPosition());
    }

    public void setReference(double setpoint, double feedforward) {
        setReference(setpoint, feedforward, getPosition());
    }

    public void setReference(double setpoint, double feedforward, double position) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;

        double pid = controller.calculate(position, setpoint);

        if(controller.atSetpoint())
            device.setPower(0.0);
        else
            device.setPower(pid + feedforward);
    }

    public double getVelocity() {
        if(encoder == null || encoder.getStatus().equals(HardwareStatus.MISSING)) return 0.0;
        return encoder.getVelocity();
    }

    public double getPosition() {
        if(encoder == null || encoder.getStatus().equals(HardwareStatus.MISSING)) return 0.0;
        return encoder.getCurrentPosition();
    }

    public void setPosition(double position) {
        if(encoder == null || encoder.getStatus().equals(HardwareStatus.MISSING)) return;
        encoder.zero(position);
    }
}