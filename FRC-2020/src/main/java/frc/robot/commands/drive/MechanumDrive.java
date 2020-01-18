/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.Mechanum_Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MechanumDrive implements Command {

  private Mechanum_Drivetrain drive;
  private XboxController xbox;

  public MechanumDrive(Mechanum_Drivetrain drive, XboxController controller) {
    this.drive = drive;
    xbox = controller;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }


  // Called repeatedly when this Command is scheduled to run

  public void execute() {
      drive.driveCartesian(xbox.getX(Hand.kRight), ((xbox.getX(Hand.kLeft)/3)*-1), ((xbox.getY(Hand.kLeft)/3)*-1));
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
