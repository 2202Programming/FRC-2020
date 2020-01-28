/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Mechanum_Drivetrain;


public class DriveWithLidarToDistanceCmd extends CommandBase {
  final double mm2in = 1.0/25.4;

  private Mechanum_Drivetrain drive;
  private Lidar_Subsystem     lidar;

  private double stopDist;   //inches
  private double acclLimit = 1.0/10.0;   //limit acceleration to factor of maxspeed


  private double  Kp=0.01, Ki=0.0, Kd=0.0;
  private ProfiledPIDController  distancePIDController;
  private TrapezoidProfile.Constraints rateLimits;
  private TrapezoidProfile.State goal;

  /**
   * Creates a new DriveWithLidarToDistanceCmd.
   * 
   * stopDistance = inches to stop from the wall
   * maxSpeed =  percent max speed (+- 1.0 max)   ### not physical units -hard to do with mechanum
   *             
   * 
   *    * 
   */
  public DriveWithLidarToDistanceCmd(Mechanum_Drivetrain drive, Lidar_Subsystem lidar, double stopDist, double maxSpeed) {
    this.drive = drive;
    this.lidar = lidar;
    this.stopDist = stopDist;

    // ratelimit [inch/s, in/s^2]
    rateLimits = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed / acclLimit );

    // create the PID with vel and accl limits 
    distancePIDController = new ProfiledPIDController(Kp, Ki, Kd, rateLimits);  


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lidar);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePIDController.reset(measurement);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read Lidar range
    double range = lidar.getAverageRange()*mm2in;
    double speedCmd = distancePIDController.calculate(range);




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePIDController.atSetpoint();
  }
}
