package frc.robot.util.misc;

/**
 * Common helper functions
 */
public class MathUtil {

    public static double limit(double x, double min, double max) {
        return Math.max(min, Math.min(x, max));
    }

    public static int limit(int x, int min, int max) {
        return Math.max(min, Math.min(x, max));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    public static double applyDeadband(final double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

}
