/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.VelocityDrive;

public class DriveDistanceCmd extends CommandBase {
  final VelocityDrive drive;
  double distRight;          //ft
  double distLeft;           //ft

  /**
   * Creates a new DriveDistanceCmd.
   */
  public DriveDistanceCmd(VelocityDrive driveTrain, double distLeft, double distRight) {
    this.drive = driveTrain;
    this.distLeft  = distLeft;
    this.distRight = distRight;
  }

  public DriveDistanceCmd(VelocityDrive driveTrain, double dist) {
    this(driveTrain, dist, dist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
