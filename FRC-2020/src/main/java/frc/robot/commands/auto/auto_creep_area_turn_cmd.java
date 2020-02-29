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
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class auto_creep_area_turn_cmd extends CommandBase {
  //Turn only based on limelight angle

  private final VelocityDifferentialDrive_Subsystem drive;
  private final Limelight_Subsystem limelight;
  private double angleTarget;
  //private double targetArea;
  private double Kap = 2, Kai = 0.00, Kad = 0.02; // angle drive PIDs
  private final PIDController anglePIDController;
  private double angleToleranceDeg = 3;
  private double maxAngleRate;
  //private double maxSpeed;
  private double current_angle;
  private double kDegreesToDPS = 1; // convert PID rotation output to degrees per second for
                                    // VelocityDifferentalDrive

  //private boolean forward;

  public auto_creep_area_turn_cmd(final VelocityDifferentialDrive_Subsystem drive, final Limelight_Subsystem limelight,
      final double angleTarget, final double maxAngleRate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.limelight = limelight;
    this.angleTarget = angleTarget;
    this.maxAngleRate = maxAngleRate;

    anglePIDController = new PIDController(Kap, Kai, Kad);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Robot.command = "Auto Limelight Turn";

    limelight.enableLED();
    drive.resetPosition();

    anglePIDController.reset();
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);
    anglePIDController.setSetpoint(angleTarget);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // angle pid to limelight angle
    current_angle = limelight.getFilteredX();
    double angleCmd = kDegreesToDPS * anglePIDController.calculate(current_angle);
    angleCmd = MathUtil.clamp(angleCmd, -maxAngleRate, maxAngleRate);

    SmartDashboard.putNumber("Max Angle Rate", maxAngleRate);
    SmartDashboard.putNumber("Filtered Angle", current_angle);
    SmartDashboard.putNumber("PID Output DPS", angleCmd);

    // move rotation only
    drive.velocityArcadeDrive(0, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    Robot.command = "None";
    drive.resetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (Math.abs(current_angle - angleTarget) < angleToleranceDeg);
  }
}
