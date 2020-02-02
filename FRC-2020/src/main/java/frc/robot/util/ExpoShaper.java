package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpiutil.math.MathUtil;

public class ExpoShaper {
    // expo only works on normalize inputs
    double kExpo; // 0.0 to 1.0 for flatness 0 -> straight curve
    double kCexpo; // complement to expo
    DoubleSupplier inFunct = null;

    // deadzone for normalzied
    double deadband = 0.0;

    public ExpoShaper(double kExpo, DoubleSupplier inFunct) {
        setExpo(kExpo);
        this.inFunct = inFunct;
    }

    public ExpoShaper(double kExpo) {
        this(kExpo, null);
    }

    public void setExpo(double a) {
        if (a > 1.0)
            a = 1.0;
        if (a < 0.0)
            a = 0.0;
        kExpo = a;
        kCexpo = 1.0 - a;
    }

    public void setDeadzone(double dz) {
        deadband = MathUtil.clamp(dz, -1.0, 1.0);
    }

    // use Gord W's expo function
    public double expo(double x) {
        return x * x * x * kExpo + kCexpo * x;
    }

    // create a DoubleSupplier to use
    public double get() {
        double x = applyDeadband(inFunct.getAsDouble());
        return expo(x);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value) {
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
