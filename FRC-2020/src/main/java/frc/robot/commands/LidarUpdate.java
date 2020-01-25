/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drive.LidarDrive;
import frc.robot.subsystems.Lidar_Subsystem;

public class LidarUpdate extends CommandBase {

  private Lidar_Subsystem m_lidarsubsystem;
  private LidarDrive m_lidardrive;

  public LidarUpdate(Lidar_Subsystem x, LidarDrive y) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_lidardrive = y;
    m_lidarsubsystem = x;

  }

  // Called repeatedly when this Command is scheduled to ru
  public void execute() {
    m_lidarsubsystem.updateLidar();
    
    if (m_lidarsubsystem.getAverageRange() < 500){
     m_lidardrive.x_speed = 0;
    } else m_lidardrive.x_speed = 0.1;

    m_lidarsubsystem.printLog();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      Set<Subsystem> subs = new HashSet<Subsystem>();
      subs.add(m_lidarsubsystem);
      return subs;
  }
}
