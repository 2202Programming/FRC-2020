/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;

import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ifx.ArcadeDrive;

public class auto_limelightDrive_cmd extends CommandBase {
  
   final double mm2in = 1.0 / 25.4;

  private final ArcadeDrive drive;
  private final Limelight_Subsystem limelight;
  private final Lidar_Subsystem lidar;

  private final double stopDist; // inches
  private double tolerancePct = .05;
  private double angleToleranceDeg = 3;
  private double kDegreesToDPS = 1; //convert PID rotation output to degrees per second for VelocityDifferentalDrive
  private double maxSpeed;
  private double angleTarget;
  private final double Kp = 0.2, Ki = 0.04, Kd = 0.25;
  private double Kap = 0.1, Kai = 0.001, Kad = 0.0; //angle drive PIDs
  private final PIDController distancePIDController;
  private final PIDController anglePIDController;
  private double targetForwardPower;

  /**
   * Creates a new DriveWithLidarToDistanceCmd.
   * 
   * stopDistance = inches to stop from the wall maxSpeed = percent max speed (+-
   * 1.0 max)
   * angleTarget = degrees from front of robot to target
   * Recommend using this command with withTimeout()
   * 
   * D Laufenberg
   * 
   */
  public auto_limelightDrive_cmd(final ArcadeDrive drive, final Limelight_Subsystem limelight, final Lidar_Subsystem lidar,
      final double stopDist, final double angleTarget, final double maxSpeed, double targetForwardPower) {
    this.drive = drive;
    this.limelight = limelight;
    this.lidar = lidar;
    this.stopDist = stopDist; //inches
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;
    this.targetForwardPower = targetForwardPower;

    // create the PID with vel and accl limits
    distancePIDController = new PIDController(Kp, Ki, Kd);
    anglePIDController = new PIDController(Kap, Kai, Kad);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);
  }

  public auto_limelightDrive_cmd(ArcadeDrive drive, Limelight_Subsystem limelight, final Lidar_Subsystem lidar,
  double stopDist, double maxSpeed, double angleTarget,
      double tolerancePct, double targetForwardPower) {
    this(drive, limelight, lidar, stopDist, maxSpeed, angleTarget, targetForwardPower);
    this.tolerancePct = tolerancePct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableLED();
    distancePIDController.reset();
    distancePIDController.setSetpoint(stopDist);
    distancePIDController.setTolerance(stopDist * tolerancePct, 0.5);
    distancePIDController.setIntegratorRange(0, 3);

    anglePIDController.reset();
    anglePIDController.setSetpoint(angleTarget);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);

    Robot.command = "Auto limelight drive";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double target_angle = limelight.getX();
    double angleCmd = kDegreesToDPS * anglePIDController.calculate(target_angle);
    angleCmd = MathUtil.clamp(angleCmd, -maxSpeed, maxSpeed);


   // SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
    SmartDashboard.putNumber("Angle", target_angle);
    SmartDashboard.putNumber("PID Output DPS", angleCmd);

    SmartDashboard.putData(anglePIDController);
  
    // move rotation only
    drive.arcadeDrive(targetForwardPower, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    limelight.disableLED();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lidar.valid();
  }
}
