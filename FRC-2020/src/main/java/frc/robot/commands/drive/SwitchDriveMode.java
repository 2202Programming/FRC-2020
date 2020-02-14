package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.TankDrive;

public class SwitchDriveMode extends InstantCommand {
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  private final DriverControls dc;
  private boolean isArcade = false;

  public SwitchDriveMode(VelocityDifferentialDrive_Subsystem driveTrain, DriverControls dc) {
    this.driveTrain = driveTrain;
    this.dc = dc;

  }

  public void initialize() {
    if (isArcade) {
      CommandScheduler.getInstance().setDefaultCommand(driveTrain, new TankDriveCmd(dc, driveTrain));
    } else {
      CommandScheduler.getInstance().setDefaultCommand(driveTrain, new ArcadeDriveCmd(dc, driveTrain));
    }

    isArcade = !isArcade;
  }
}