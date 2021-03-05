// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
//fix to use -> import frc.robot.subsystems.ifx.VelocityDrive;

public class GyroHeadingCompensator extends CommandBase {
  final double Wmax = 15.0;  // limit compensation rotation rate [deg/s]
  final double MaxError = 5.0; // [deg] expect error to get no bigger than this
  final double Kp = Wmax/MaxError;    //[deg/s] [1/deg-error] = [1/s]

  VelocityDifferentialDrive_Subsystem drive;
  Rotation2d init_rotation;
  
  // update every frame
  double m_correction;
  
  /** Creates a new GyroHeadingCompensator
   *  This doesn't correct for user commanded rotation, yet
   *  but is an example of feeding a compensation signal into
   *  the drive train as a rotation command.
  */
  public GyroHeadingCompensator(VelocityDifferentialDrive_Subsystem drive) {
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_rotation = drive.getPose().getRotation();
    drive.setHeadingCompensator(this::correction);
    m_correction = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d current_rotation = drive.getPose().getRotation();
    double errDegrees =  init_rotation.getDegrees() - current_rotation.getDegrees();

    m_correction = errDegrees * Kp;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setHeadingCompensator(null); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // correction function called by drive train
  double correction() {
    return m_correction;
  }

}
