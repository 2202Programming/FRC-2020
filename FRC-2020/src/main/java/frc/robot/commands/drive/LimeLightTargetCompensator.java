// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class LimeLightTargetCompensator extends CommandBase {
  
  // code stolen from auto_limelightTurnToShoot
  final double max_rot_rate = 5.0; // [deg/sec]
  final double Kap = 0.015, Kai = 0.00001, Kad = 3.0; //angle drive PIDs
  final double angleToleranceDeg = 1.0;

  final VelocityDifferentialDrive_Subsystem drive;
  final Limelight_Subsystem limelight;
  private final PIDController anglePIDController;
  Rotation2d init_rotation;

  // update every frame [degree/sec]
  double m_correction;    // sent to Drivetrain
  double m_target_angle;  //measured from LL
   

  /** Creates a new LimeLightTargetCompensator. */
  public LimeLightTargetCompensator() {
    this.drive = RobotContainer.getInstance().driveTrain;
    limelight = RobotContainer.getInstance().limelight;
    
   // create the PID with vel and accl limits
   anglePIDController = new PIDController(Kap, Kai, Kad);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_correction = 0.0;
    init_rotation = drive.getPose().getRotation();
    anglePIDController.reset();
    anglePIDController.setSetpoint(0);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);

    drive.setHeadingCompensator(this::correction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    double angleRateCmd = 0.0;   // [deg/s]
    // run PID on Limelight
    if (limelight.valid()) {
      m_target_angle = limelight.getX();    //goes to zero (our setpoint) when aligned
      angleRateCmd = anglePIDController.calculate(m_target_angle);
    }
    else {
      anglePIDController.reset();
    }
    m_correction = MathUtil.clamp(angleRateCmd, -max_rot_rate, max_rot_rate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.disableLED();
    m_correction=0.0;
    drive.setHeadingCompensator(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // correction function called by drive train - degrees/s rotation request
  double correction() {
    return m_correction;  
  }

  boolean isOnTarget() {
    return anglePIDController.atSetpoint();
  }

}
