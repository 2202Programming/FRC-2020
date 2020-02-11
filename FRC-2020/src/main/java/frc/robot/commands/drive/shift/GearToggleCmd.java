package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GearShifter;

public class GearToggleCmd extends CommandBase {
  /**
   * Creates a new ToggleGearCmd.
   * 
   * This could be an instant, but gearshifting may get more complex
   * so used full command base.
   * 
   */
  private GearShifter m_shifter;
  private boolean m_inLowGear = true; // gets set to true on first call

  public GearToggleCmd(GearShifter shifter) {
    m_shifter = shifter;
    addRequirements(shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_inLowGear = !m_inLowGear;
    if (m_inLowGear = true) {
      m_shifter.shiftUp();
    } else {
      m_shifter.shiftDown();
    }
    m_inLowGear = !m_inLowGear;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
