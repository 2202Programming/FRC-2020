package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeToggleCmd extends InstantCommand {
  private final Intake_Subsystem m_intake;
  @SuppressWarnings("unused")
  private double m_magMotor;
  private double m_intakeMotor;

  public IntakeToggleCmd(final Intake_Subsystem intake, double magPower, double intakePower) {
    m_intake = intake;
    m_magMotor = magPower;
    m_intakeMotor = intakePower;
  }

  // Called when the command is initially scheduled, only once for instant commands.
  @Override
  public void initialize() {
    if (m_intake.intakeIsOn()) {
      // m_intake.magazineOff();
      m_intake.intakeOff();
    }
    else {
      //  m_intake.magazineOn(m_magMotor);
      m_intake.intakeOn(m_intakeMotor);
    }
  }
}