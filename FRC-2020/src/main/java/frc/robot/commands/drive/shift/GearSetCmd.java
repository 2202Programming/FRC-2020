package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.Shifter.Gear;
import frc.robot.subsystems.ifx.VelocityDrive;

public class GearSetCmd extends InstantCommand {
  /**
   * Command toggles the gear on the drive train.
   * 
   * Turned to an instant because the shifter/drivetrain will handle
   * the details of the up/down request.
   * 
   */
  final VelocityDrive m_drive;
  //final Shifter m_shifter;
  final Gear m_desiredGear;

  public GearSetCmd(VelocityDrive drive, Gear gear ) {
    m_drive = drive;
    //m_shifter = shifter;
    m_desiredGear = gear;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
   // if (m_shifter.getCurrentGear() == m_desiredGear) return;
    if (m_desiredGear == Gear.HIGH)
      m_drive.reqShiftUp();
    else 
      m_drive.reqShiftDown();
    
  }

}
