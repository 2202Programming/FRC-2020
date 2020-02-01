package frc.robot.util;

import java.util.function.DoubleSupplier;

public class ExpoShaper {
     // expo only works on normalize inputs
    double kExpo;          // 0.0 to 1.0 for flatness 0 -> straight curve
    double kCexpo;         // complement to expo 
    DoubleSupplier         inFunct=null;

    public ExpoShaper(double kExpo, DoubleSupplier inFunct) {
        setExpo(kExpo);
        this.inFunct = inFunct;
    }

    public ExpoShaper(double kExpo){
        this(kExpo, null);
    }

    public void setExpo(double a) {
        if (a > 1.0) a = 1.0;
        if (a < 0.0) a = 0.0;
        kExpo = a;
        kCexpo = 1.0 - a;
    }

    // use Gord W's expo function
     public double expo(double x) {
        return x*x*x*kExpo + kCexpo*x;   
    }

    //create a DoubleSupplier to use
    public double get() {
        double x = inFunct.getAsDouble();
        return expo(x);
    }


}

        
    