package org.firstinspires.ftc.teamcode.utility.math;

import java.util.List;


/**
 * Copied from Stephen's VectorMath
 */
public class ListMath {
    // Summation
    static public double sum(List<Double> v) {
        double ret = 0.0;
        for (int i = 0; i < v.size(); ++i) {
            ret += v.get(i);
        }
        return ret;
    }

    // Get the average value of an entire List
    static public double average(List<Double> v) {
        if (v.size() == 0)
            return 0.0;
        return sum(v) / v.size();
    }

    // Get the average over some past number of steps
    static public double average(List<Double> v, int steps) {
        int size = v.size();
        if (steps > size) {
            steps = size;
        }
        int start = size - steps;

        double ret = 0.0;
        for (int i = start; i < size; ++i) {
            ret += v.get(i);
        }

        //ret /= size - steps;
        ret /= steps;

        return ret;
    }

    // Get the standard deviation of entire List
    static public double standardDeviation(List<Double> v) {
        int size = v.size();
        double mean = average(v);
        double acc = 0.0;
        for (int i = 0; i < size; ++i) {
            acc += Math.pow(v.get(i) - mean,2);
        }
        double variance = acc / size;
        return Math.sqrt(variance);
    }

    // Get the standard deviation of some past number of steps
    static public double standardDeviation(List<Double> v, int steps) {
        int size = v.size();
        if (steps > size) {
            steps = size;
        }
        double mean = average(v,steps);
        double acc = 0.0;
        for (int i = size-steps; i < size; ++i) {
            acc += Math.pow(v.get(i) - mean,2);
        }
        double variance = acc / steps;
        return Math.sqrt(variance);
    }

    // Get the span of the List, i.e. lastValue - firstValue.
    static public double span(List<Double> v) {
        if (v.size() > 0)
            return v.get(v.size()-1) - v.get(0);
        return 0.0;
    }

    public static void addRemoveN(List<Double> v, double value, double  maxSize) {
        v.add(value);
        while (v.size() > maxSize) {
            v.remove(v.get(0));
        }
    }


    // Derivative
    // x is the independent variable, y is dependent. Returns derivative for unequally spaced points. xest is the x value at which to evaluate the derivative.
    // x and y Lists don't need to be the same length if they have three elements or more each
    static public double derivative(List<Double> x, List<Double> y, double xest) {
        int nx = x.size();
        int ny = y.size();
        if (x.size() < 2 || y.size() < 2) {
            return 0.0;
        } else if (nx == 2 || ny == 2) {
            return (y.get(1) - y.get(0)) / (x.get(1) - x.get(0));
        }
        double x2 = x.get(nx - 1);
        double x1 = x.get(nx - 2);
        double x0 = x.get(nx - 3);

        double y2 = y.get(ny - 1);
        double y1 = y.get(ny - 2);
        double y0 = y.get(ny - 3);

        // dydx
        return y0 * (2.0 * xest - x1 - x2) / ((x0 - x1) * (x0 - x2))
                + y1 * (2.0 * xest - x0 - x2) / ((x1 - x0) * (x1 - x2))
                + y2 * (2.0 * xest - x0 - x1) / ((x2 - x0) * (x2 - x1));
    }

    // Default derivative computation at the latest x value
    static public double derivative(List<Double> x, List<Double> y) {
        return derivative(x, y, x.get(x.size()-1));
    }

    // x is the independent variable, y is dependent. Returns the integral estimate.
    static public double integral(List<Double> x, List<Double> y) {
        // Uses the composite trapezoidal rule to calculate the integral for unequally spaced data.
        if (x.size() != y.size()) {
            throw new RuntimeException("ERROR: Lists to be integrated are of differing size") ;
        }

        if (x.size() < 2)
            return 0.0;

        int n = x.size();
        int end = n - 1; // must end one step short because we are looking ahead one step in the loop
        double I = 0.0;

        for (int i = 0; i < end; ++i)
            I += (x.get(i + 1) - x.get(i)) * (y.get(i + i) + y.get(i)) / 2.0;

        return I;
    }
}