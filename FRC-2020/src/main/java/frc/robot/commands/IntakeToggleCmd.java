package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeToggleCmd extends InstantCommand {
  private boolean intakeUp = true;
  private final Intake_Subsystem m_intake;
  private double m_magMotor = 0.8;

  public IntakeToggleCmd(final Intake_Subsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    //No reqs.,  Only needs the piston controls
    m_intake = intake;
    m_intake.raiseIntake();  //force solnoid to match, even if nothing happens
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeUp)  {
      m_intake.raiseIntake();
      m_intake.magazineOff();
    }
    else {
      m_intake.lowerIntake();
      m_intake.magazineOn(m_magMotor);
    }
    //toggle the state
    intakeUp = !intakeUp;
  }
}