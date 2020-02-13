/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

/**
 * Creates a new IntakeOn.
 */
public class IntakeOn extends InstantCommand {

  private static Intake_Subsystem m_intake;
  private double magMotorPower = 0.7;
  private double intakeMotorPower = 0.75;

  public IntakeOn(Intake_Subsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
  //  addRequirements(m_intake);
    IntakeOn.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intake.intakeIsOn()) {
      m_intake.intakeOff();
      m_intake.magazineOff();
    } else {
      m_intake.intakeOn(intakeMotorPower);
      m_intake.magazineOn(magMotorPower);
    }
  }
}
