/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class auto_ball_capture_cmd extends CommandBase {
  /**
   * Creates a new auto_ball_capture_cmd.
   */

   private final Intake_Subsystem intake;
   private double intake_strength;
   private double magazine_strength;
   private final VelocityDifferentialDrive_Subsystem drive;
   private final Limelight_Subsystem limelight;
   private double drive_speed;


  public auto_ball_capture_cmd(Intake_Subsystem intake, VelocityDifferentialDrive_Subsystem drive, Limelight_Subsystem limelight,
  double intake_strength, double magazine_strength, double drive_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake; 
    this.intake_strength = intake_strength;
    this.magazine_strength = magazine_strength;
    this.drive = drive;
    this.limelight = limelight;
    this.drive_speed = drive_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.lowerIntake();
    intake.intakeOn(intake_strength);
    intake.magazineOn(magazine_strength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.velocityArcadeDrive(drive_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    drive.velocityArcadeDrive(0, 0);
    limelight.disableLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
