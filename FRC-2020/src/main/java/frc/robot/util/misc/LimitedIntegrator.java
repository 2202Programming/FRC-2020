package frc.robot.util.misc;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;

/**
 * LimitedIntegrator integrates input subject to rate limits
 * 
 * 1 in --> K -- RL --> - ---> PL ---> out s RL = rate limits, PL = positiion
 * limits
 * 
 * This is useful to ingetrate a joystick so you get a total position command.
 * That way when you let off the joystick, the position being commanded, X,
 * stays.
 * 
 * Using a rate-command is very common.
 * 
 */
public class LimitedIntegrator {
  double dT; // sample period seconds - use robots in most cases

  // dx in device units of setter/getter functions
  double dx_fall; // falling dx/dt (device units)/second
  double dx_raise; // raising dx/dt (device units)/second

  // dead zone is in physical units, after kRate gain is applied
  // max is right hand side
  double dz_mag; // dead zone magnitude (after gain)
  double kGain = 1.0; // K gain, example applied to joystick

  // Position limits
  double x_max; // max command value for device (device units)
  double x_min; // min command value for device (device units)
  final DoubleSupplier inFunct; // function input X

  double in; // obtained from inFunct each frame

  // output vars
  double X; // output value (device units)
  double Xprev; // previous frame output (state)

  public LimitedIntegrator(final Double dT, final DoubleSupplier inFunct, // typical joystick function would go here
      double kGain, double x_min, double x_max, double dx_fall, double dx_raise) {
    this.dT = dT;
    this.inFunct = inFunct;
    this.kGain = kGain;
    this.x_max = x_max;
    this.x_min = x_min;
    this.dx_fall = dx_fall;
    this.dx_raise = dx_raise;
    setDeadZone(0.0);
    initialize();
  }

  // Output
  public double get() {
    return X;
  }

  public void setGain(double k) {
    kGain = k;
  }

  // designed to be called from an FRC Command if needed
  public void initialize() {
    Xprev = 0.0;
    X = 0.0;
  }

  // jumps to given output, good for setting inital position
  public void setX(double pos) {
    X = MathUtil.clamp(pos, x_min, x_max);
    Xprev = X;
  }

  // designed to be called from FRC Command if needed, call once per frame
  // and no more because it does an integration in Rate mode.
  public void execute() {
    in = inFunct.getAsDouble();

    // deadzone and rate limit the input
    double cmdDz = deadZone(in * kGain);
    double dX = MathUtil.clamp(cmdDz, dx_fall, dx_raise);

    // integrate the dX desired rate limited X
    double x = Xprev + dX * dT;

    // position limit the output
    Xprev = X;
    X = MathUtil.clamp(x, x_min, x_max);
  }

  // set dz values and compute correcting scales so we get max deflections
  public void setDeadZone(double dz_mag) {
    this.dz_mag = Math.abs(dz_mag);
  }

  // apply deadzone to input in and scale to get propper range
  private double deadZone(double x) {
    if (Math.abs(x) < dz_mag)
      return (double) 0.0;
    return x;
  }

  public void setConstraints(double kGain, double x_min, double x_max, double dx_fall, double dx_raise) {
    this.kGain = kGain;
    this.x_max = x_max;
    this.x_min = x_min;
    this.dx_fall = dx_fall;
    this.dx_raise = dx_raise;
  }
}
