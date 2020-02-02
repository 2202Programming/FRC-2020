/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.DriverControls;
/**
 * HID_Subsystem - Human Input Device
 * 
 * Use this to bind specific devices to their uses.
 */
public class HID_Subsystem extends SubsystemBase implements DriverControls {
  /**
   * Creates a new HID_Subsystem.
   */
  public HID_Subsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public double getVelocitX() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocityY() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocity() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getRotation() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean isNormalized() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void bindButtons() {
    // TODO Auto-generated method stub

  }
}
