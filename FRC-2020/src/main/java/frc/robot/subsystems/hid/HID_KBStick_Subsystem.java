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
import frc.robot.util.ExpoShaper;

public class HID_KBStick_Subsystem extends SubsystemBase implements DriverControls {
  KBSimStick driver;
  //only one stick for now, so no tank drive

  ExpoShaper velShaper;
  ExpoShaper rotShaper;

  double vel, z_rot;

  // invertGain is used to change the controls for driving backwards easily.
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  /**
   * Creates a new HID_KBStick_Subsystem.
   */
  public HID_KBStick_Subsystem(double velExpo, double rotExpo) {
    driver = (KBSimStick) registerController(Id.Driver, new KBSimStick(Id.Driver.value));

    velShaper = new ExpoShaper(velExpo, () -> driver.getAxis(Axis.kY));
    rotShaper = new ExpoShaper(rotExpo, () -> (driver.getAxis(Axis.kRot) * -1.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vel = velShaper.get() * invertGain;
    z_rot = rotShaper.get() * invertGain;

  }

  // TODO: fix non-arcade drive functions
  @Override
  public double getVelocityX() {
    return 0;
  }

  @Override
  public double getVelocityY() {
    return 0;
  }

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
    // TODO Auto-generated method stub
    return 0;
  }

}