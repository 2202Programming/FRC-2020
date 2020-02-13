/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class ShooterOn extends InstantCommand {
  private Intake_Subsystem m_intake;
  private final double shootPower = 0.7;

  public ShooterOn(Intake_Subsystem m_intake) {
    this.m_intake = m_intake;

    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.shooterIsOn()) {
      m_intake.shooterOff();
    } else {
      m_intake.shooterOn(shootPower);
    }
  }
}
