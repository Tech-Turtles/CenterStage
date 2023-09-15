package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;

/**
 * Class for Rev Expansion hubs or Control hubs.
 */
public class ExpansionHub extends HardwareDevice {

    private LynxModule device;
    private LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

    public ExpansionHub(String configName) {
        super(configName, LynxModule.class);
    }

    @Override
    public void initialize(Object device) {
        if(!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.device = (LynxModule) device;
        this.device.setBulkCachingMode(bulkCachingMode);

        setStatus(HardwareStatus.SUCCESS);
    }

    public ExpansionHub configureBulkCachingMode(LynxModule.BulkCachingMode bulkCachingMode) {
        this.bulkCachingMode = bulkCachingMode;
        return this;
    }

    public LynxModule.BulkCachingMode getBulkCachingMode() {
        return bulkCachingMode;
    }

    public void setBulkCachingMode(LynxModule.BulkCachingMode bulkCachingMode) {
        if(getStatus().equals(HardwareStatus.MISSING)) return;

        this.bulkCachingMode = bulkCachingMode;
        device.setBulkCachingMode(bulkCachingMode);
    }

    public void clearBulkCache() {
        if(getStatus().equals(HardwareStatus.MISSING)) return;

        device.clearBulkCache();
    }
}