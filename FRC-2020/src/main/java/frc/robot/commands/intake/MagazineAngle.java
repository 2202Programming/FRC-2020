// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineAngle extends CommandBase {
  // manual up/down at const motor speed or to an exact position
  public enum Direction {Up, Down, ToPositon};
  static final double RPM = 3;

  final MagazinePositioner  magPositioner; 
  final double degrees;
  final Direction direction;

  /** Creates a new MagazineAngle. */
  public MagazineAngle(Intake_Subsystem intake, Direction dir) {
    this.magPositioner = intake.getMagazine().getMagPositioner();
    this.degrees = 0.0;
    this.direction = dir;
    addRequirements(magPositioner);
  }

  public MagazineAngle(Intake_Subsystem intake, double degrees) {
    this.magPositioner = intake.getMagazine().getMagPositioner();
    this.degrees = degrees;
    this.direction=Direction.ToPositon;
    addRequirements(magPositioner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (direction) {
      case Up:
              magPositioner.wind(RPM);
              break;
      case Down:
              magPositioner.wind(-RPM);
              break;
      case ToPositon:
            magPositioner.setAngle(degrees);
            break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // no matter how we got here, manual, or ToPosition, call it home.
    magPositioner.stopAndHold(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if ( direction == Direction.ToPositon) return magPositioner.isAtSetpoint();
   return false;  // manual never done
  }
}
