/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimbArmExtension extends InstantCommand {
  private ClimberSubsystem climber;
  private boolean extended;
  /**
   * Creates a new ClimbArmExtend.
   */
  public SetClimbArmExtension(ClimberSubsystem climber, boolean extended) {
    this.climber = climber;
    this.extended = extended;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (extended) {
      climber.extendArm();
    } else {
      climber.retractArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

}
