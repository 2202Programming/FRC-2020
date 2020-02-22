package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DualDrive;

public class SwitchDriveMode extends InstantCommand {
  public final DualDrive driveTrain;
  private boolean isArcade = false;
  private final ArcadeDriveCmd arcadeAuto;
  private final TankDriveCmd tankAuto;

  public SwitchDriveMode(final DualDrive driveTrain, final ArcadeDriveCmd arcadeAuto, final TankDriveCmd tankAuto) {
    this.driveTrain = driveTrain;
    this.arcadeAuto = arcadeAuto;
    this.tankAuto = tankAuto;
  }

  public void initialize() {
    if (isArcade) {
        driveTrain.setDefaultCommand(tankAuto);
        arcadeAuto.cancel();
    } else {
        driveTrain.setDefaultCommand(arcadeAuto);
        tankAuto.cancel();
    }
    isArcade = !isArcade;
  }
}