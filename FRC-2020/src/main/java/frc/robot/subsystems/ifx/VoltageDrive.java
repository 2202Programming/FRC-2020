// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ifx;


import edu.wpi.first.wpilibj2.command.Subsystem;

/** Simple voltage driven robot */
public interface VoltageDrive extends Odometry, Subsystem{

  public void tankDriveVolts(double leftVolts, double rightVolts);  
}
