/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimbArmRotation extends InstantCommand {
  private ClimberSubsystem climber;
  private double speed;
  /**
   * Creates a new SetClimbArmRotation.
   */
  public SetClimbArmRotation(ClimberSubsystem climber, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setRotationSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setRotationSpeed(0);
  }

}
