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
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;

public class auto_drive_lidar_until_limelight extends CommandBase {
  //drive a set lidar angle at fixed speed until limelight is available

  final double mm2in = 1.0 / 25.4;

  private final ArcadeDrive drive;
  private final Lidar_Subsystem lidar;
  private final Limelight_Subsystem limelight;

  private double angleToleranceDeg = 3;
  private double kDegreesToPerPower = -1;
  private double maxSpeed;
  private double angleTarget;
  private final double Kap = 0.05, Kai = 0.001, Kad = 0.0;
  private final PIDController anglePIDController;

  public auto_drive_lidar_until_limelight(final ArcadeDrive drive, final Lidar_Subsystem lidar, final Limelight_Subsystem limelight,
  final double angleTarget, final double maxSpeed) {
    this.drive = drive;
    this.lidar = lidar;
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;
    this.limelight = limelight;

    // create the PID with vel and accl limits
    anglePIDController = new PIDController(Kap, Kai, Kad);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.lidar);
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    anglePIDController.reset();
    anglePIDController.setSetpoint(angleTarget);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);

    Robot.command = "Drive with lidar";

    limelight.enableLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // read Lidar rangr

        double angleCmd = kDegreesToPerPower * anglePIDController.calculate(lidar.findAngle());
        angleCmd = MathUtil.clamp(angleCmd, -maxSpeed, maxSpeed);
    
        SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
        SmartDashboard.putNumber("Angle", lidar.findAngle());
        SmartDashboard.putNumber("PID Output (%) (Angle)", angleCmd);

        drive.arcadeDrive(maxSpeed, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.valid();
  }
}
