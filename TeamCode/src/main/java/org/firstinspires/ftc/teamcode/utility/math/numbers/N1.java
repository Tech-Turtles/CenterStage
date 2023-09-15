package org.firstinspires.ftc.teamcode.utility.math.numbers;

import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N1 extends Num implements Nat<N1> {



    private N1() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 1;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N1 instance = new N1();
}
