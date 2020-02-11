/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;

public class MagazineAdjust extends CommandBase {
  private static Intake_Subsystem intake;
  private boolean forward;
  private static final double strength = 0.7;
  /**
   * Creates a new MagazineAdjust.
   */
  public MagazineAdjust(Intake_Subsystem intake, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.forward = forward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (forward) intake.magazineOn(strength);
    else intake.magazineOn(-strength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.magazineOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
