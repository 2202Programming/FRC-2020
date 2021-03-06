/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class DriveWithLimelightToDistanceDegCmd extends CommandBase {
  final double mm2in = 1.0 / 25.4;

  private final VelocityDifferentialDrive_Subsystem drive;
  private final Limelight_Subsystem limelight;

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
  public DriveWithLimelightToDistanceDegCmd(final VelocityDifferentialDrive_Subsystem drive, final Limelight_Subsystem limelight,
      final double stopDist, final double angleTarget, final double maxSpeed) {
    this.drive = drive;
    this.limelight = limelight;
    this.stopDist = stopDist; //inches
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;

    // create the PID with vel and accl limits
    distancePIDController = new PIDController(Kp, Ki, Kd);
    anglePIDController = new PIDController(Kap, Kai, Kad);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);
  }

  public DriveWithLimelightToDistanceDegCmd(VelocityDifferentialDrive_Subsystem drive, Limelight_Subsystem limelight, double stopDist, double maxSpeed, double angleTarget,
      double tolerancePct) {
    this(drive, limelight, stopDist, maxSpeed, angleTarget);
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
    drive.velocityArcadeDrive(0, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    limelight.disableLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}