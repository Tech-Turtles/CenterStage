package org.firstinspires.ftc.teamcode.utility.math.numbers;

import org.firstinspires.ftc.teamcode.utility.math.Nat;
import org.firstinspires.ftc.teamcode.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N7 extends Num implements Nat<N7> {



    private N7() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 7;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N7 instance = new N7();
}
