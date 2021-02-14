// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.ux.DriverPreferences;

public class TankVelDriveCmd extends CommandBase {
    VelocityDrive drive;
    DriverControls dc;

    // config read at init, possible from Dashboard
    double vMax;   //[ft/s]
    double rotMax; //[deg/s]
    
    // outputs to drive
    double vl;
    double vr;
  
    public TankVelDriveCmd(final DriverControls dc, final VelocityDrive drive) {
        this.drive = drive;
        this.dc = dc;
        addRequirements(drive);
    }

  @Override
  public void initialize() {
    super.initialize();

     // get vMax from Dashboard here
    DriverPreferences dp = RobotContainer.getInstance().dashboard.getDriverPreferences();
    vMax = dp.getMaxSpeed();
    rotMax = dp.getMaxRotation();
  }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vMax = drive.getMaxVelocity();
    rotMax = drive.getMaxRotation();

    // read controls in normalize units +/- 1.0, scale to physical units
    vl = dc.getVelocityLeft() * vMax;
    vr = dc.getVelocityRight() * vMax;
    
    drive.velocityTankDrive(vl, vr);
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }
}