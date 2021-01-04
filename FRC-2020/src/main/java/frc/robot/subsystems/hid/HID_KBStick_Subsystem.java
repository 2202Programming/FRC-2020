/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hid.KBSimStick.Axis;
import frc.robot.subsystems.ifx.DriverControls;

public class HID_KBStick_Subsystem extends SubsystemBase implements DriverControls {
  KBSimStick driver;
  //only one stick for now, so no tank drive
  ExpoShaper velShaper;
  ExpoShaper rotShaper;

   //XYRot / Swerve (field or robot relative)
   ExpoShaper velXShaper;    // left/right  
   ExpoShaper velYShaper;    // forward/backward 
   ExpoShaper xyRotShaper;   // rotation for XYRot

   //values updated each frame
  double vel, z_rot;         //arcade
  double velLeft, velRight;  //tank
  double velX,velY, xyRot;   //XTRot

  // invertGain is used to change the controls for driving backwards easily.
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  int initialDriverButtons;

  /**
   * Creates a new HID_KBStick_Subsystem.
   */
  public HID_KBStick_Subsystem(double velExpo, double rotExpo) {
    driver = (KBSimStick) registerController(Id.Driver, new KBSimStick(Id.Driver.value));
    initialDriverButtons = getButtonsRaw(Id.Driver);

    velShaper = new ExpoShaper(velExpo, () -> driver.getAxis(Axis.kThrottle));
    rotShaper = new ExpoShaper(rotExpo, () -> -driver.getAxis(Axis.kRot));

    // XYRot or Swerve Drive
    // Rotation on Left-X axis,  X-Y throttle on Right
    velXShaper = new ExpoShaper(velExpo, () -> driver.getAxis(Axis.kX));     
    velXShaper = new ExpoShaper(velExpo, () -> driver.getAxis(Axis.kY));
    xyRotShaper = new ExpoShaper(rotExpo, () -> -driver.getAxis(Axis.kRot));  //inverted rot

    initialDriverButtons = getButtonsRaw(Id.Driver);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Arcade 
    vel = velShaper.get() * invertGain;
    z_rot = rotShaper.get() * invertGain;
    
     //XYRot
     velX = velXShaper.get();
     velY = velYShaper.get();
     xyRot = xyRotShaper.get();

    // tank hack
    velLeft = vel;
    velRight = velY;
  }

  /**
   * XYRotation controls
   */
  @Override
  public double getVelocityX() {
    return velX;
  }

  @Override
  public double getVelocityY() {
    return velY;
  }
 
  public double getXyRot() {
     return xyRot;
  }

  @Override
  public double getXYRotation() {
    return xyRot;
  }

  /**
   * Tank Controls - throttle = left, kY = right side (kind of a one stick hack for tank)
   */
  @Override
  public double getVelocityLeft() {
    return vel;
  }

  @Override
  public double getVelocityRight() {
    return vel;
  }

  @Override
  public double getVelocity() {
    return vel;
  }

  @Override
  public double getRotation() {
   return z_rot;
  }

  @Override
  public boolean isNormalized() {
    return false;
  }

  @Override
  public void setInvertControls(boolean invert) {
    invertGain = (invert) ? -1.0 : 1.0;
  }

  @Override
  public boolean isControlInverted() {
    if (invertGain < 0.0 ) return true;
    return false;
  }

  @Override
  public int getInitialButtons(Id id) {
    if (id == Id.Driver) return initialDriverButtons;
    return  0;
  }

}