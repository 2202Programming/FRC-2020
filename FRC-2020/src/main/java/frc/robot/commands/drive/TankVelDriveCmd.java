// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.VelocityDrive;

public class TankVelDriveCmd extends CommandBase {
    final VelocityDrive drive;
    final DriverControls dc;

    // config read at init, possible from Dashboard
    double vMax;   //[ft/s]
    //double rotMax; //[deg/s]

    /**
     *  Constant velocity driver controls
     *  */
    static class DumbDriver implements DriverControls {
      double left_vel;
      double right_vel;
      DumbDriver(double vl, double vr) { 
        this.left_vel = vl;
        this.right_vel = vr;
      }
      
      @Override
      public boolean isNormalized() {  return false;   }
      @Override
      public double getVelocityLeft() { return left_vel;  }
      @Override
      public double getVelocityRight() { return right_vel;  }
    }
  
    public TankVelDriveCmd(final DriverControls dc, final VelocityDrive drive) {
        this.drive = drive;
        this.dc = dc;
        addRequirements(drive);
    }

    /**
     * TankVelDriveCmd - useful for simple motions or keeping watchdog happy when 
     *    not moving.
     * 
     * @param drive
     * @param vel_left
     * @param vel_right
     */
    public TankVelDriveCmd(final VelocityDrive drive, final double vel_left, final double vel_right) {
      this(new DumbDriver(vel_left, vel_right), drive);
    }


    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if normalized, scale with max vel, otherwise used incomming vel.
    vMax = (dc.isNormalized()) ? drive.getMaxVelocity() : 1.0;
    //rotMax = drive.getMaxRotation();

    // read controls in normalize units +/- 1.0, scale to physical units   
    double vl = dc.getVelocityLeft() * vMax;
    double vr = dc.getVelocityRight() * vMax;
    
    //velocity drive train takes physical units
    drive.velocityTankDrive(vl, vr);
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }
}