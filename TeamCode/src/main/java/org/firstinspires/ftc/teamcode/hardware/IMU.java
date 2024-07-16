package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation3d;
import org.firstinspires.ftc.teamcode.utility.misc.AxesSigns;
import org.firstinspires.ftc.teamcode.utility.misc.BNO055IMUUtil;

import java.util.Optional;

//ToDo Add IMU axes order and signs as a constant in RobotConstants
//ToDo Add configure & set methods for axes mapping
public class IMU extends HardwareDevice {

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private BNO055IMUImpl device;
    private Thread imuThread;
    private volatile boolean isStopRequested = false;

//    private Rotation3d offset = new Rotation3d();
//    private Rotation3d current = new Rotation3d();
    private Rotation2d offset = new Rotation2d();
    private Rotation2d current = new Rotation2d();

    public IMU(String configName) {
        super(configName, BNO055IMUImpl.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }
        this.device = (BNO055IMUImpl) device;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.device.initialize(parameters);

        // If your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(this.device, AxesOrder.XYZ, AxesSigns.NPN);

        imuThread = new Thread(() -> {
            while (isStopRequested) {
                synchronized (imuLock) {
                    current = Rotation2d.fromRadians(this.device.getAngularOrientation().firstAngle);
                }
            }
        });
        imuThread.start();

        setStatus(HardwareStatus.SUCCESS);
    }

    public void setOffset(Rotation2d offset) {
        this.offset = offset;
    }

    public Rotation2d getRawRotation() {
        return current;
    }

    public Rotation2d getRotation() {
        return getRawRotation().minus(offset);
    }

    public Optional<Translation3d> getAccel() {
        if(getStatus().equals(HardwareStatus.MISSING)) return Optional.empty();
        Acceleration a = device.getAcceleration();
        return Optional.of(
                new Translation3d(a.xAccel, a.yAccel, a.zAccel).times(9.81));
    }

    public Double getXAngularVelocity() {
        if(getStatus().equals(HardwareStatus.MISSING)) return 0.0;
        return (double) device.getAngularVelocity().xRotationRate;
    }

    public void stop() {
        isStopRequested = true;
        imuThread.interrupt();
    }
}