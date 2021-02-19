/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakePosition extends InstantCommand {
  public enum Direction {
    Up, Down, Toggle
  }

  private Intake_Subsystem m_intake;
  Direction direction;

  public IntakePosition(Intake_Subsystem m_intake, IntakePosition.Direction direction) {
    this.m_intake = m_intake;
    this.direction = direction;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Direction cmd = direction;
    if (direction == Direction.Toggle) {
        cmd = m_intake.isIntakeUp() ? Direction.Down : Direction.Up;
    }
    switch (cmd) {
      case Up:
        m_intake.raiseIntake();
        m_intake.intakeOff();
      break;
      
      case Down:
        m_intake.lowerIntake();
        break;
      default:
    }
  }
}
