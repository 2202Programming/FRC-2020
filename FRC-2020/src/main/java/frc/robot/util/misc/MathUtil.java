package frc.robot.util.misc;

/**
 *  Common helper functions
 */
public class MathUtil {

    public static double limit(double x, double min, double max) {
        return Math.max(min, Math.min(x, max) );
    }

    public static int limit(int x, int min, int max) {
       return Math.max(min, Math.min(x,max));
    }

}
