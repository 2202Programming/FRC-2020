/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;

/**
 * Creates a new IntakeOn.
 */
public class IntakeOn extends CommandBase {

  private static Intake_Subsystem m_intake;
  private double motorPower = 0.7;

  public IntakeOn(Intake_Subsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
  //  addRequirements(m_intake);
    IntakeOn.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeOn(RPM_TARGET);
    m_intake.magazineOn(motorPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intakeOff();
    m_intake.magazineOff();
    System.out.println("IntakeOn-Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
