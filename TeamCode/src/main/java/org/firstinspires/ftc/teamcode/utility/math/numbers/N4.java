package org.firstinspires.ftc.teamcode.utility.math.numbers;

import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N4 extends Num implements Nat<N4> {



    private N4() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 4;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N4 instance = new N4();
}
