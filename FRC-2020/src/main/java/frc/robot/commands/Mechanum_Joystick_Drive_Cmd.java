/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.Mechanum_Drivetrain;


public class Mechanum_Joystick_Drive_Cmd extends CommandBase {
  /**
   * Creates a new Mechanum_Joystick_Drive_Cmd.
   */

  private Mechanum_Drivetrain drive;
  private XboxController xbox;
  public double x_speed = 0.1;


  public Mechanum_Joystick_Drive_Cmd(Mechanum_Drivetrain drive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    xbox = controller;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveCartesian(xbox.getX(Hand.kRight), ((xbox.getX(Hand.kLeft)/3)*-1),((xbox.getY(Hand.kLeft) / 3) * -1));
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
