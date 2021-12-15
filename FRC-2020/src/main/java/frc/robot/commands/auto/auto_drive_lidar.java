/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class auto_drive_lidar extends CommandBase {
  final double mm2in = 1.0 / 25.4;

  private final VelocityDifferentialDrive_Subsystem drive;
  private final Lidar_Subsystem lidar;

  private final double stopDist; // mm
  private final double tolerance = 50; // mm
  private double angleToleranceDeg = 3;
  private double kInchesToPerPower = 1;
  private double kDegreesToPerPower = -1;
  private double maxSpeed;
  private double angleTarget;
  private double range;
  private boolean forwards;
  private final double Kp = 2, Ki = 0.04, Kd = 0.25;
  private final double Kap = 1, Kai = 0.001, Kad = 0.0;
  private final PIDController distancePIDController;
  private final PIDController anglePIDController;
  private double maxAngleSpeed = 60;

  public auto_drive_lidar(final VelocityDifferentialDrive_Subsystem drive, final Lidar_Subsystem lidar, final double stopDist,
      final double maxSpeed, final boolean forwards, final double angleTarget) {
    this.drive = drive;
    this.lidar = lidar;
    this.stopDist = stopDist; // mm
    this.maxSpeed = maxSpeed;
    this.forwards = forwards;
    this.angleTarget = angleTarget;

    kInchesToPerPower = -2;

    // create the PID with vel and accl limits
    distancePIDController = new PIDController(Kp, Ki, Kd);
    anglePIDController = new PIDController(Kap, Kai, Kad);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.lidar);
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePIDController.reset();
    distancePIDController.setSetpoint(stopDist);
    distancePIDController.setTolerance(tolerance, 0.5);
    distancePIDController.setIntegratorRange(0, 3);

    Robot.command = "Auto Drive with lidar";

    anglePIDController.reset();
    anglePIDController.setSetpoint(angleTarget);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    range = lidar.getAverageRange();
    double speedCmd = distancePIDController.calculate(range) * kInchesToPerPower;
    double angleCmd = kDegreesToPerPower * anglePIDController.calculate(lidar.findAngle());
    speedCmd = MathUtil.clamp(speedCmd, -maxSpeed, maxSpeed);
    angleCmd = MathUtil.clamp(angleCmd, -maxAngleSpeed, maxAngleSpeed);

    SmartDashboard.putNumber("PID error (mm)", distancePIDController.getPositionError());
    SmartDashboard.putNumber("PID Verr", distancePIDController.getVelocityError());
    SmartDashboard.putNumber("Range (mm)", range);
    SmartDashboard.putNumber("PID Output (FPS)", speedCmd);

    SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
    SmartDashboard.putNumber("Angle", lidar.findAngle());
    SmartDashboard.putNumber("PID Output (DPS) (Angle)", angleCmd);

    SmartDashboard.putNumber("Range-Dist (mm)", (range - stopDist));
    SmartDashboard.putNumber("Tolerance (mm)", tolerance);

    drive.velocityArcadeDrive(speedCmd, angleCmd);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    Robot.command = "None";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (forwards) { //moving forward, so end when Current range - target distance is < tolerance
      if ((range - stopDist) <= (tolerance))
        return true;
      else
        return false;
    }
    else { //moving backwards, so end when target distance - Current range is < tolerance
      if ((stopDist - range) <= (tolerance))
        return true;
      else
        return false;
    }
  }
}