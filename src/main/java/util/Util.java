package util;

import java.util.Arrays;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        if(v > max) {
            v = max;
        }
        if(v < min) {
            v = min;
        }
        return v;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean singleEpsilonEquals(double a, double epsilon) {
        return -epsilon < a && a < epsilon;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double[] absLimitWithRatio(double[] in, double magnitude) {
        double[] copy = Arrays.copyOf(in, in.length);
        double largest = 0;
        boolean limit = false;
        for(double d : copy) {
            limit |= Math.abs(d) > magnitude;
            largest = Math.max(largest, Math.abs(d));
        }

        if(limit) {
            for (int i = 0; i < copy.length; i++) {
                copy[i] /= largest;
                copy[i] *= magnitude;
            }
        }
        return copy;
    }

    public static double angleBetween(double a, double b) {
        double diff = (a - b + Math.PI*2*10) % (Math.PI*2);
        return diff <= Math.PI ? diff : Math.PI*2 - diff;
    }

    public static boolean isTurnCCW(double curRad, double wantedRad) {
        double diff = wantedRad - curRad;        // CCW = counter-clockwise ie. left
        return diff > 0 ? diff > Math.PI : diff >= -Math.PI;
    }

    public static double normalizeAngle(double rad) {
        while(rad > 2*Math.PI) {
            rad -= Math.PI * 2;
        }
        while(rad < 0) {
            rad += Math.PI * 2;
        }
        return rad;
    }
}
