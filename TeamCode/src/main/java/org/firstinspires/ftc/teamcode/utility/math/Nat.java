package org.firstinspires.ftc.teamcode.utility.math;

import org.firstinspires.ftc.teamcode.utility.math.numbers.N1;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N10;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N2;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N3;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N4;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N5;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N6;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N7;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N8;
import org.firstinspires.ftc.teamcode.utility.math.numbers.N9;

public interface Nat<T extends Num> {
    /**
     * The number this interface represents.
     *
     * @return The number backing this value.
     */
    int getNum();

    static Nat<N1> N1() {
        return N1.instance;
    }

    static Nat<N2> N2() {
        return N2.instance;
    }

    static Nat<N3> N3() {
        return N3.instance;
    }

    static Nat<N4> N4() {
        return N4.instance;
    }

    static Nat<N5> N5() {
        return N5.instance;
    }

    static Nat<N6> N6() {
        return N6.instance;
    }

    static Nat<N7> N7() {
        return N7.instance;
    }

    static Nat<N8> N8() {
        return N8.instance;
    }

    static Nat<N9> N9() {
        return N9.instance;
    }

    static Nat<N10> N10() {
        return N10.instance;
    }
}
