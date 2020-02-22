/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.ArcadeDrive;
import frc.robot.subsystems.ifx.DriverControls;

public class ArcadeVelDriveCmd extends CommandBase {
  DriverControls dc;
  ArcadeDrive drive;

  //max speeds allowed
  double vMax;    //fps
  double rotMax;  //deg per sec

  /**
   * Creates a new ArcadeVelDriveCmd.
   */
  public ArcadeVelDriveCmd(DriverControls dc, ArcadeDrive driveTrain, double velMaxFps, double rotMaxDps) {
    this.dc = dc;
    this.drive = driveTrain;
    this.vMax = velMaxFps;
    this.rotMax = rotMaxDps;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read controls in normalize units +/- 1.0
    double v = dc.getVelocity();
    double rot = dc.getRotation();

    // scale inputs based on max commands
    v *=vMax;
    rot *=rotMax;

    drive.velocityArcadeDrive(v, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }
}
