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

public class auto_creep_cmd extends CommandBase {
  /**
   * Creates a new auto_creep_cmd.
   */

  private final ArcadeDrive drive;
  private final Limelight_Subsystem limelight;
  private double angleTarget;
  private double targetDistance;
  private double Kap = 0.2, Kai = 0.01, Kad = 0.0; //angle drive PIDs
  private double Kp = 0.5, Ki = 0.001, Kd = 0.0; //distance drive PIDs
  private final PIDController anglePIDController;
  private final PIDController distancePIDController;
  private double tolerancePct = .05;
  private double angleToleranceDeg = 3;
  private double maxAngleRate;
  private double maxSpeed;
  private double kDegreesToDPS = 1; //convert PID rotation output to degrees per second for VelocityDifferentalDrive
  

  public auto_creep_cmd(final ArcadeDrive drive, final Limelight_Subsystem limelight, final double angleTarget, final double maxSpeed, final double maxAngleRate, final double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.limelight = limelight;
    this.targetDistance = targetDistance; //feet
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;
    this.maxAngleRate = maxAngleRate;

    anglePIDController = new PIDController(Kap, Kai, Kad);
    distancePIDController = new PIDController(Kp, Ki, Kd);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableLED();
    drive.resetPosition();

    distancePIDController.reset();
    distancePIDController.setSetpoint(targetDistance);
    distancePIDController.setTolerance((targetDistance) * tolerancePct, 0.5);
    //distancePIDController.setIntegratorRange(0, 3);

    anglePIDController.reset();
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);
    anglePIDController.setSetpoint(angleTarget);

    Robot.command = "Auto creep";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //angle pid to limelight angle
    double current_angle = limelight.getX();
    double angleCmd = kDegreesToDPS * anglePIDController.calculate(current_angle);
    angleCmd = MathUtil.clamp(angleCmd, -maxAngleRate, maxAngleRate);

    //distanace pid
    double current_position = (drive.getLeftPos()+drive.getRightPos())/2;
    double speedCmd = distancePIDController.calculate(current_position);
    speedCmd = MathUtil.clamp(speedCmd, -maxSpeed, maxSpeed);

   // SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
    SmartDashboard.putNumber("Max Angle Rate", maxAngleRate);
    SmartDashboard.putNumber("Angle", current_angle);
    SmartDashboard.putNumber("PID Output DPS", angleCmd);
    //SmartDashboard.putData(anglePIDController);

    SmartDashboard.putNumber("Max Speed", maxSpeed);
    SmartDashboard.putNumber("Target Distance", targetDistance);
    SmartDashboard.putNumber("Distance", current_position);
    SmartDashboard.putNumber("PID Output Distance", speedCmd);
    //SmartDashboard.putData(distancePIDController);
  
    // move rotation only
    drive.arcadeDrive(0, angleCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    limelight.disableLED();
    Robot.command = "None";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    double current_position = (drive.getLeftPos()+drive.getRightPos())/2;
    double current_travel_distance = current_position - starting_position;
    if (targetDistance == current_travel_distance)
      return true;
    else return false;
    */
    return !limelight.valid();
  }
}
