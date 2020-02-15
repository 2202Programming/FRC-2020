/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake_Subsystem;

public class ToggleIntakeRaised extends InstantCommand {
  private boolean intakeUp = true;

  private Intake_Subsystem m_intake;
  public ToggleIntakeRaised(Intake_Subsystem m_intake) {
    addRequirements(m_intake);
    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (intakeUp) {
      m_intake.lowerIntake();
    } else {
      m_intake.raiseIntake();
    }

    //flip state
    intakeUp = !intakeUp;

    Robot.command = "Toggle intake raised";
  }
}
