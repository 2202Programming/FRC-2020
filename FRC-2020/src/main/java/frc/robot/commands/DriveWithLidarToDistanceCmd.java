/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Mechanum_Drivetrain;

public class DriveWithLidarToDistanceCmd extends CommandBase {
  final double mm2in = 1.0 / 25.4;

  private final Mechanum_Drivetrain drive;
  private final Lidar_Subsystem lidar;

  private final double stopDist; // inches
  private double tolerancePct = .05;

  private final double Kp = 0.01, Ki = 0.0, Kd = 0.0;
  private final PIDController distancePIDController;

  /**
   * Creates a new DriveWithLidarToDistanceCmd.
   * 
   * stopDistance = inches to stop from the wall maxSpeed = percent max speed (+-
   * 1.0 max)
   * 
   * Recommend using this command with withTimeout()
   * 
   * D Laufenberg
   * 
   */
  public DriveWithLidarToDistanceCmd(final Mechanum_Drivetrain drive, final Lidar_Subsystem lidar,
      final double stopDist, final double maxSpeed) {
    this.drive = drive;
    this.lidar = lidar;
    this.stopDist = stopDist;

    // create the PID with vel and accl limits
    distancePIDController = new PIDController(Kp, Ki, Kd);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lidar);
    addRequirements(drive);
  }

  public DriveWithLidarToDistanceCmd(Mechanum_Drivetrain drive, Lidar_Subsystem lidar, double stopDist, double maxSpeed,
      double tolerancePct) {
    this(drive, lidar, stopDist, maxSpeed);
    this.tolerancePct = tolerancePct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePIDController.reset();
    distancePIDController.setSetpoint(stopDist);
    distancePIDController.setTolerance(stopDist * tolerancePct);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read Lidar range
    double range = lidar.getAverageRange() * mm2in;
    double speedCmd = distancePIDController.calculate(range);

    // move forward, no rotation
    drive.driveCartesian(speedCmd, 0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePIDController.atSetpoint();
  }
}
