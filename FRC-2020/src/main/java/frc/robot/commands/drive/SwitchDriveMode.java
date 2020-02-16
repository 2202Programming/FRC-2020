package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DualDrive;

public class SwitchDriveMode extends InstantCommand {
  public final DualDrive driveTrain;
  private boolean isArcade = false;
  private final Command tankAuto;
  private final Command arcadeAuto;

  public SwitchDriveMode(final DualDrive driveTrain, final Command arcadeAuto, final Command tankAuto) {
    this.driveTrain = driveTrain;
    this.arcadeAuto = arcadeAuto;
    this.tankAuto = tankAuto;
  }

  public void initialize() {
    if (isArcade) {
        driveTrain.setDefaultCommand(tankAuto);
    } else {
        driveTrain.setDefaultCommand(arcadeAuto);
    }
    isArcade = !isArcade;
  }
}