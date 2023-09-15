package org.firstinspires.ftc.teamcode.utility.math.kinematics;

import org.firstinspires.ftc.teamcode.utility.math.interpolation.Interpolatable;

public interface WheelPositions<T extends WheelPositions<T>> extends Interpolatable<T> {
    /**
     * Returns a copy of this instance.
     *
     * @return A copy.
     */
    T copy();
}