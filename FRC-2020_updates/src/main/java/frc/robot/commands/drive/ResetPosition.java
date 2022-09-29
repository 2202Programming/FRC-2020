/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.Odometry;

public class ResetPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Odometry m_subsystem;
  private final Pose2d m_pose;
  /**
   * Creates a new Reset.
   */
  public ResetPosition(Odometry subsystem, Pose2d pose2d) {
    // Requirements not needed for resetting counters
    m_subsystem = subsystem;
    m_pose = pose2d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    //m_subsystem.resetPosition();
    m_subsystem.resetOdometry(m_pose);
  }

  @Override
  public boolean runsWhenDisabled() { return true;}

}
