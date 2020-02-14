package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeToggleCmd extends InstantCommand {
  private boolean intakeOn = false;
  private final Intake_Subsystem m_intake;
  private double m_magMotor;
  private double m_intakeMotor;

  public IntakeToggleCmd(final Intake_Subsystem intake, double magPower, double intakePower) {
    // Use addRequirements() here to declare subsystem dependencies.
    //No reqs.,  Only needs the piston controls
    m_intake = intake;
    m_intake.raiseIntake();  //force solnoid to match, even if nothing happens
    m_magMotor = magPower;
    m_intakeMotor = intakePower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeOn) {
      m_intake.magazineOff();
      m_intake.intakeOff();
    }
    else {
      m_intake.magazineOn(m_magMotor);
      m_intake.intakeOn(m_intakeMotor);

    }
    //toggle the state
    
    intakeOn = !intakeOn;
  }
}