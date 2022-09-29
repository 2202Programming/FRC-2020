/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import static frc.robot.subsystems.Intake_Subsystem.SAFE_INTAKE_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class IntakePosition extends CommandBase {
  public enum Direction {
    Up, Down, Toggle
  }

  private Intake_Subsystem m_intake;
  MagazinePositioner m_mag_postioner;
  Command m_magToSafePostion;
  Direction m_direction_request;
  Direction m_dir; // real direciton, accounts for toggle
  boolean m_done;

  public IntakePosition(Intake_Subsystem m_intake, IntakePosition.Direction direction) {
    this.m_intake = m_intake;
    this.m_mag_postioner = m_intake.getMagazine().getMagPositioner();
    this.m_direction_request = direction;
    this.m_magToSafePostion = new MagazineAngle(m_intake, SAFE_INTAKE_ANGLE - 2);

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_done = false;
    m_dir = m_direction_request;
    if (m_direction_request == Direction.Toggle) {
      m_dir = m_intake.isIntakeUp() ? Direction.Down : Direction.Up;
    }
    // if we are raising the intake, the Magazine position must be lowered, so we have to
    // wait for that.
    if (m_dir == Direction.Up && m_mag_postioner.get() > SAFE_INTAKE_ANGLE) {
      m_magToSafePostion.schedule();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    switch (m_dir) {
      case Up:
        // we may have to wait for mag to come down
        if (m_mag_postioner.get() > SAFE_INTAKE_ANGLE) {
          return;
        }

        // made it here, safe to move the intake
        m_intake.raiseIntake();
        m_intake.intakeOff();
        m_done = true;
        break;

      case Down:
        m_intake.lowerIntake();
        m_done = true;
        break;
      default:
    }
  }

  @Override
  public boolean isFinished() {
    return m_done;
  }
}
