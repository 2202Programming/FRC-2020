/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;

public class auto_drive_lidar extends CommandBase {
  final double mm2in = 1.0 / 25.4;

  private final ArcadeDrive drive;
  private final Lidar_Subsystem lidar;

  private final double stopDist; // inches
  private double tolerancePct = .05;
  private double angleToleranceDeg = 3;
  private double kInchesToPerPower = 1;
  private double kDegreesToPerPower = -1;
  private double maxSpeed;
  private double angleTarget;
  private double range;
  private final double Kp = 0.2, Ki = 0.04, Kd = 0.25;
  private final double Kap = 0.05, Kai = 0.001, Kad = 0.0;
  private final PIDController distancePIDController;
  private final PIDController anglePIDController;


  public auto_drive_lidar(final ArcadeDrive drive, final Lidar_Subsystem lidar, final double stopDist,
  final double angleTarget, final double maxSpeed) {
    this.drive = drive;
    this.lidar = lidar;
    this.stopDist = stopDist; // inches
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;

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
    distancePIDController.setTolerance(stopDist * tolerancePct, 0.5);
    distancePIDController.setIntegratorRange(0, 3);

    anglePIDController.reset();
    anglePIDController.setSetpoint(angleTarget);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);

    Robot.command = "Auto Drive with lidar";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    range = lidar.getAverageRange() * mm2in;
    double speedCmd = distancePIDController.calculate(range) * kInchesToPerPower;
    double angleCmd = kDegreesToPerPower * anglePIDController.calculate(lidar.findAngle());
    speedCmd = MathUtil.clamp(speedCmd, -maxSpeed, maxSpeed);
    angleCmd = MathUtil.clamp(angleCmd, -maxSpeed, maxSpeed);

    SmartDashboard.putNumber("PID error (inches)", distancePIDController.getPositionError());
    SmartDashboard.putNumber("PID Verr", distancePIDController.getVelocityError());
    SmartDashboard.putNumber("Range (inches)", range);
    SmartDashboard.putNumber("PID Output (%)", speedCmd);

    SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
    SmartDashboard.putNumber("Angle", lidar.findAngle());
    SmartDashboard.putNumber("PID Output (%) (Angle)", angleCmd);

    drive.arcadeDrive(speedCmd, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((range < (stopDist + stopDist*tolerancePct)) && (range > (stopDist - stopDist*tolerancePct))) //finish when range is within tolerance of stopDist
    return true;
    else return false;
  }
}
