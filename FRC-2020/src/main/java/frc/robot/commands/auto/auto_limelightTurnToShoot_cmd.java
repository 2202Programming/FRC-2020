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
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;

public class auto_limelightTurnToShoot_cmd extends CommandBase {
  
   final double mm2in = 1.0 / 25.4;

  private final ArcadeDrive drive;
  private final Limelight_Subsystem limelight;

  private double angleToleranceDeg = 1;
  private double kDegreesToDPS = 1; //convert PID rotation output to degrees per second for VelocityDifferentalDrive
  private double maxSpeed;
  private double Kap = 0.015, Kai = 0.000, Kad = 3.0; //angle drive PIDs
  private final PIDController anglePIDController;
  private double target_angle;


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
  public auto_limelightTurnToShoot_cmd(final ArcadeDrive drive, final Limelight_Subsystem limelight, final double maxSpeed) {
    this.drive = drive;
    this.limelight = limelight;
    this.maxSpeed = maxSpeed;

    // create the PID with vel and accl limits
    anglePIDController = new PIDController(Kap, Kai, Kad);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableLED();

    anglePIDController.reset();
    anglePIDController.setSetpoint(0);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);

    Robot.command = "Auto limelight shoot";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target_angle = limelight.getX();
    double angleCmd = kDegreesToDPS * anglePIDController.calculate(target_angle);
    angleCmd = MathUtil.clamp(angleCmd, -maxSpeed, maxSpeed);


    SmartDashboard.putNumber("Angle", target_angle);
    SmartDashboard.putNumber("PID Output DPS", angleCmd);

    SmartDashboard.putData(anglePIDController);
  
    // move rotation only
    drive.arcadeDrive(0, -angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    limelight.disableLED();
    drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(target_angle) < angleToleranceDeg) && (Math.abs(drive.getLeftVel(false))<0.05);
  }
}
