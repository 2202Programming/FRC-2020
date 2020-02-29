/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class DriveOnTrajectory extends SequentialCommandGroup {
  private VelocityDifferentialDrive_Subsystem driveTrain;
  private Trajectory path;
  private RamseteController ramseteController;

  private double ramB = 0.9;
  private double ramZ = 0.9;
  /**
   * Creates a new DriveOnTrajectory.
   */
  public DriveOnTrajectory(VelocityDifferentialDrive_Subsystem driveTrain, Trajectory path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.path = path;
    addRequirements(driveTrain);
    
    ramseteController = new RamseteController(ramB, ramZ);

    addCommands(new RamseteCommand(
      path, driveTrain::getPose, ramseteController, driveTrain.DRIVE_KINEMATICS, driveTrain::tankDrive, driveTrain));
  }
}
