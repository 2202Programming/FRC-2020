package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DualDrive;
/**
 * Swtiches the drivetrain between two different driveing modes
 * 
 */
public class SwitchDriveMode extends InstantCommand {
  public final DualDrive driveTrain;
  private boolean isCmd1 = false;
  private final Command cmd1;
  private final Command cmd2;

  public SwitchDriveMode(final DualDrive driveTrain, final Command cmd1, final Command cmd2) {
    this.driveTrain = driveTrain;
    this.cmd1 = cmd1;
    this.cmd2 = cmd2;
  }

  public void initialize() {
    Command oldCmd = driveTrain.getDefaultCommand();
    if (isCmd1) {
        driveTrain.setDefaultCommand(cmd2);
    } else {
        driveTrain.setDefaultCommand(cmd1);
    }
    // should allow switchout of any default being used
    oldCmd.cancel();
    
    //update state
    isCmd1 = !isCmd1;
  }
}