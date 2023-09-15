package org.firstinspires.ftc.teamcode.utility.math.numbers;

import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N9 extends Num implements Nat<N9> {



    private N9() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 9;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N9 instance = new N9();
}
