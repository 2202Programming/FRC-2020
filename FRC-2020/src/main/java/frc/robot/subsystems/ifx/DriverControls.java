/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriverControls extends Subsystem {
  
  //mech, tank
  public double getVelocitX();
  public double getVelocityY();

  //archive drive
  public double getVelocity();
  public double getRotation();
  
  //physical or normalize values
  boolean isNormalized();

  // called during initialization
  public void bindButtons();
  
  
}