package frc.robot.subsystems.hid;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

public class ExpoShaper {
    // expo only works on normalize inputs
    double kExpo; // 0.0 to 1.0 for flatness 0 -> straight curve
    double kCexpo; // complement to expo
    DoubleSupplier inFunct = null;

    // deadzone for normalzied
    double deadband = 0.0;

    public ExpoShaper(final double kExpo, final DoubleSupplier inFunct) {
        setExpo(kExpo);
        this.inFunct = inFunct;
    }

    public ExpoShaper(final double kExpo) {
        this(kExpo, null);
    }

    public ExpoShaper setExpo(final double a) {
        kExpo = MathUtil.clamp(a, 0.0, 1.0);
        kCexpo = 1.0 - kExpo;
        return this;
    }

    public ExpoShaper setDeadzone(final double dz) {
        deadband = MathUtil.clamp(dz, 0.0, 0.10);
        return this;
    }

    // use Gord W's expo function
    public double expo(final double x) {
        return x * x * x * kExpo + kCexpo * x;
    }

    // create a DoubleSupplier to use
    public double get() {
        return  expo(applyDeadband(inFunct.getAsDouble()));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(final double value) {
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
