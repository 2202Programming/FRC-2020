package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DualDrive;

public class SwitchDriveMode extends InstantCommand {
  public final DualDrive driveTrain;
  private boolean isArcade = false;
  private final Command tankCmd;
  private final Command arcadeCmd;

  public SwitchDriveMode(final DualDrive driveTrain, final Command arcadeCmd, final Command tankCmd) {
    this.driveTrain = driveTrain;
    this.arcadeCmd = arcadeCmd;
    this.tankCmd = tankCmd;
  }

  public void initialize() {
    if (isArcade) {
      driveTrain.setDefaultCommand(tankCmd);
    } else {
      driveTrain.setDefaultCommand(arcadeCmd);
    }
    isArcade = !isArcade;
  }
}