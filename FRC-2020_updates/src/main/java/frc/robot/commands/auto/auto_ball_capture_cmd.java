/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class auto_ball_capture_cmd extends CommandBase {
  //open loop driving with intake running

   private final Intake_Subsystem intake;
   private double intake_strength;
   private double magazine_strength;
   private final VelocityDifferentialDrive_Subsystem drive;
   private double drive_speed;


  public auto_ball_capture_cmd(Intake_Subsystem intake, VelocityDifferentialDrive_Subsystem drive,
  double intake_strength, double magazine_strength, double drive_speed) {

    this.intake = intake; 
    this.intake_strength = intake_strength;
    this.magazine_strength = magazine_strength;
    this.drive = drive;
    this.drive_speed = drive_speed;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.lowerIntake();
    intake.intakeOn(intake_strength);
    intake.getMagazine().beltOn(magazine_strength);
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
    intake.getMagazine().beltOff();
    drive.velocityArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
