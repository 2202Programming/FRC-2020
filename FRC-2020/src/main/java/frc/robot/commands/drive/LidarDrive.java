/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import frc.robot.subsystems.Mechanum_Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LidarDrive implements Command {

  private Mechanum_Drivetrain drive;
  public double x_speed = 0.1;

  public LidarDrive(Mechanum_Drivetrain drive) {
    this.drive = drive;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }


  // Called repeatedly when this Command is scheduled to run

  public void execute() {
      drive.driveCartesian(0, 0, x_speed);
  }

  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }


  @Override
  public Set<Subsystem> getRequirements() {
      Set<Subsystem> subs = new HashSet<Subsystem>();
      subs.add(drive);
      return subs;
  }
}
