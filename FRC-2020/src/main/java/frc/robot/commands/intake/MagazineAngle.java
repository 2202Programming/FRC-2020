// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineAngle extends CommandBase {

  MagazinePositioner  magPositioner; 
  double degrees;

  /** Creates a new MagazineAngle. */
  public MagazineAngle(Intake_Subsystem intake, double degrees) {
    magPositioner = intake.getMagazine().getMagPositioner();
    this.degrees = degrees;
    addRequirements(magPositioner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    magPositioner.setAngle(degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if interrupted, don't lock.  
    magPositioner.stopAndHold(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean  done = magPositioner.isAtSetpoint();
    if (done) {
      //magPositioner.stopAndHold(false);
    }
    return done;
  }
}
