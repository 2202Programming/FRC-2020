package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakePower extends InstantCommand {
  public enum Power { Off, On, ReverseOn, Toggle}

  private final Intake_Subsystem m_intake;
  private final double m_intakeMotorPwr;
  private final Power m_power; 

  public IntakePower(final Intake_Subsystem intake, Power power, double perctPower) {
    m_intake = intake;
    m_power = power;

    // clamp and flip sign for ReverseOn
    m_intakeMotorPwr = MathUtil.clamp(perctPower, 0.0, 1.0) * ((m_power == Power.ReverseOn)  ? -1.0 : 1.0);
  }

  // Called when the command is initially scheduled, only once for instant commands.
  @Override
  public void initialize() {
    Power cmd = m_power;

    if (m_power == Power.Toggle)   {
      cmd = (m_intake.isIntakeOn()) ? Power.Off : Power.On;
    }
    
    switch (cmd) {
      case On:
      case ReverseOn:
        m_intake.intakeOn(m_intakeMotorPwr);
        break;
      
      case Off:
        m_intake.intakeOff();
        break;
      
      case Toggle:// already handled  
      default:
    }
  }
}