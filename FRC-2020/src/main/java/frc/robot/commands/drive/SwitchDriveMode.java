package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DualDrive;

public class SwitchDriveMode extends InstantCommand {
  public final DualDrive driveTrain;
  private boolean isArcade = false;
  private final ArcadeDriveCmd arcadeCmd;
  private final TankDriveCmd tankCmd;

  public SwitchDriveMode(final DualDrive driveTrain, final ArcadeDriveCmd arcadeCmd, final TankDriveCmd tankCmd) {
    this.driveTrain = driveTrain;
    this.arcadeCmd = arcadeCmd;
    this.tankCmd = tankCmd;
  }

  public void initialize() {
    if (isArcade) {
        driveTrain.setDefaultCommand(tankCmd);
        arcadeCmd.cancel();
    } else {
        driveTrain.setDefaultCommand(arcadeCmd);
        tankCmd.cancel();
    }
    isArcade = !isArcade;
  }
}