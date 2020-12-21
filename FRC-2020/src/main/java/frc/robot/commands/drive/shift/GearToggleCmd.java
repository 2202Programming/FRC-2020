package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.Shifter;

public class GearToggleCmd extends InstantCommand {
  /**
   * Command toggles the gear on the drive train.
   * 
   * Turned to an instant because the shifter/drivetrain will handle
   * the details of the up/down request.
   * 
   */
  private Shifter m_shifter;

  public GearToggleCmd(Shifter shifter) {
    m_shifter = shifter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    if (m_shifter.getCurrentGear() == Gear.LOW_GEAR) {
      m_shifter.shiftUp();
    } else {
      m_shifter.shiftDown();
    }
  }

}
