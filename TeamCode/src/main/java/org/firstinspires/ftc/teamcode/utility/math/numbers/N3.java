package org.firstinspires.ftc.teamcode.utility.math.numbers;

import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N3 extends Num implements Nat<N3> {



    private N3() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 3;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N3 instance = new N3();
}
