// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ifx;

import edu.wpi.first.networktables.EntryNotification;

/** 
 * 
*/
public interface DashboardUpdate {
  public void processDashboard(EntryNotification event);
}
