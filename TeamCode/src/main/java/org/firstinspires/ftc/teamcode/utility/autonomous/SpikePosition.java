package org.firstinspires.ftc.teamcode.utility.autonomous;

import androidx.annotation.NonNull;

/**
 * Enum to hold the possible spike values.
 * Ordinal of the enum correlate to the value of the spike place on the field.
 * Names are the spike position relative to the robot when the back of the robot is facing them.
 */
public enum SpikePosition {
    NONE,
    LEFT,
    MIDDLE,
    RIGHT;

    @NonNull
    public static SpikePosition getSpikeByOrdinal(int ordinal) {
        for(SpikePosition s : SpikePosition.values()) {
            if(s.ordinal() == ordinal)
                return s;
        }
        return NONE;
    }
}
